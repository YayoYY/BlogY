"""
Offline evaluation for a LeRobot dataset.

Architecture:
- Parse LeRobot-style config via `parser.wrap()`, including `--policy.path=...` or
  `--policy.type=... --policy.pretrained_path=...`.
- Load a `LeRobotDataset` with policy-compatible `delta_timestamps`, and optionally filter
  the evaluation to specific episodes.
- Support `select_action` and `action_chunk` inference modes.
  In `action_chunk` mode, one prediction is compared against the next `sequence_len` ground-truth
  actions from the current frame onward, and metrics are averaged over that sequence window.
- Write frame-wise results to CSV, including `observation.state`.

Examples:
python script/eval_offline_dataset_policy.py \
  --dataset.root=./datasets/pick_place \
  --dataset.repo_id=local/pick_place \
  --eval.episode=0 \
  --policy.path=./outputs/act_cuda1/checkpoints/last/pretrained_model

python script/eval_offline_dataset_policy.py \
  --dataset.root=./datasets/pick_place \
  --dataset.repo_id=local/pick_place \
  --policy.type=act \
  --policy.pretrained_path=./outputs/act/checkpoints/last/pretrained_model \
  --eval.inference_mode=action_chunk \
  --eval.sequence_len=25
"""

import csv
import json
import logging
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat
from typing import Any

import torch
from tqdm import tqdm

from lerobot import policies  # noqa: F401
from lerobot.configs import parser
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets import LeRobotDataset, resolve_delta_timestamps
from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.utils.constants import ACTION, OBS_PREFIX, OBS_STATE
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging


@dataclass
class DatasetEvalConfig:
    repo_id: str
    root: str | Path | None = None
    revision: str | None = None
    episode: int | None = None
    episodes: list[int] | None = None
    download_videos: bool = True
    video_backend: str | None = None
    return_uint8: bool = True
    tolerance_s: float = 1e-4
    force_cache_sync: bool = False

    def selected_episodes(self) -> list[int] | None:
        if self.episode is not None and self.episodes is not None:
            raise ValueError("Specify either --dataset.episode or --dataset.episodes, not both.")
        if self.episode is not None:
            return [self.episode]
        return self.episodes


@dataclass
class OfflineEvalConfig:
    csv_path: Path = Path("offline_eval.csv")
    start_index: int = 0
    stop_index: int | None = None
    max_samples: int | None = None
    log_every: int = 50
    save_every: int = 200
    default_task: str | None = None
    gt_action_index: int | None = None
    episode: int | None = None
    episodes: list[int] | None = None
    inference_mode: str = "select_action"
    sequence_len: int = 1

    def selected_episodes(self) -> list[int] | None:
        if self.episode is not None and self.episodes is not None:
            raise ValueError("Specify either --eval.episode or --eval.episodes, not both.")
        if self.episode is not None:
            return [self.episode]
        return self.episodes


@dataclass
class OfflineDatasetPolicyEvalConfig:
    dataset: DatasetEvalConfig
    policy: PreTrainedConfig | None = None
    eval: OfflineEvalConfig = field(default_factory=OfflineEvalConfig)
    rename_map: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        policy_path = parser.get_path_arg("policy")
        if policy_path:
            yaml_overrides = parser.get_yaml_overrides("policy")
            cli_overrides = parser.get_cli_overrides("policy") or []
            self.policy = PreTrainedConfig.from_pretrained(
                policy_path,
                cli_overrides=yaml_overrides + cli_overrides,
            )
            self.policy.pretrained_path = Path(policy_path)

        if self.policy is None:
            raise ValueError(
                "You must provide a policy with either --policy.path=... or --policy.type=..."
            )

        dataset_episodes = self.dataset.selected_episodes()
        eval_episodes = self.eval.selected_episodes()
        if dataset_episodes is not None and eval_episodes is not None:
            raise ValueError(
                "Specify episode filtering in either --dataset.{episode,episodes} or "
                "--eval.{episode,episodes}, not both."
            )

        if self.eval.start_index < 0:
            raise ValueError("--eval.start_index must be >= 0")
        if self.eval.stop_index is not None and self.eval.stop_index < 0:
            raise ValueError("--eval.stop_index must be >= 0")
        if self.eval.max_samples is not None and self.eval.max_samples <= 0:
            raise ValueError("--eval.max_samples must be > 0")
        if self.eval.gt_action_index is not None and self.eval.gt_action_index < 0:
            raise ValueError("--eval.gt_action_index must be >= 0")
        if self.eval.inference_mode not in {"select_action", "action_chunk"}:
            raise ValueError("--eval.inference_mode must be one of: select_action, action_chunk")
        if self.eval.sequence_len <= 0:
            raise ValueError("--eval.sequence_len must be > 0")
        if self.eval.inference_mode == "select_action" and self.eval.sequence_len != 1:
            raise ValueError("--eval.sequence_len must be 1 when --eval.inference_mode=select_action")

    @classmethod
    def __get_path_fields__(cls) -> list[str]:
        return ["policy"]


def _scalar_to_int(value: Any, *, key: str) -> int:
    if value is None:
        raise KeyError(f"Sample is missing required key: {key}")
    if torch.is_tensor(value):
        return int(value.item())
    return int(value)


def _resolve_selected_episodes(cfg: OfflineDatasetPolicyEvalConfig) -> list[int] | None:
    eval_episodes = cfg.eval.selected_episodes()
    if eval_episodes is not None:
        return eval_episodes
    return cfg.dataset.selected_episodes()


def _resolve_task(sample: dict[str, Any], cfg: OfflineEvalConfig, dataset: LeRobotDataset) -> str:
    task = sample.get("task")
    if isinstance(task, str) and task:
        return task
    if cfg.default_task:
        return cfg.default_task

    dataset_tasks = getattr(dataset.meta, "tasks", None)
    if dataset_tasks is not None and len(dataset_tasks.index) == 1:
        return str(dataset_tasks.index[0])

    return ""


def _batchify_observation(sample: dict[str, Any], task: str) -> dict[str, Any]:
    observation: dict[str, Any] = {}
    for key, value in sample.items():
        if not key.startswith(OBS_PREFIX):
            continue
        if torch.is_tensor(value):
            observation[key] = value.unsqueeze(0)
        else:
            observation[key] = torch.as_tensor(value).unsqueeze(0)

    observation["task"] = [task]
    return observation


def _resolve_gt_action_index(
    action_delta_indices: list[int] | None,
    override_index: int | None,
    temporal_dim: int,
) -> int:
    if override_index is not None:
        if override_index >= temporal_dim:
            raise ValueError(
                f"Configured gt_action_index={override_index} exceeds action chunk length {temporal_dim}."
            )
        return override_index

    if not action_delta_indices:
        return 0

    if 0 in action_delta_indices:
        index = action_delta_indices.index(0)
        if index < temporal_dim:
            return index

    for index, delta in enumerate(action_delta_indices):
        if delta >= 0 and index < temporal_dim:
            return index

    return 0


def _extract_gt_action(
    action_value: Any,
    action_delta_indices: list[int] | None,
    override_index: int | None,
) -> torch.Tensor:
    action = action_value if torch.is_tensor(action_value) else torch.as_tensor(action_value)
    action = action.to(dtype=torch.float32)

    if action.ndim <= 1:
        return action.reshape(-1)

    gt_index = _resolve_gt_action_index(action_delta_indices, override_index, action.shape[0])
    return action[gt_index].reshape(-1)


def _ensure_action_sequence(action_value: Any) -> torch.Tensor:
    action = action_value if torch.is_tensor(action_value) else torch.as_tensor(action_value)
    action = action.to(dtype=torch.float32)

    if action.ndim == 0:
        return action.reshape(1, 1)
    if action.ndim == 1:
        return action.unsqueeze(0)
    if action.ndim == 2:
        return action
    if action.ndim == 3 and action.shape[0] == 1:
        return action.squeeze(0)
    raise ValueError(f"Unsupported action tensor shape: {tuple(action.shape)}")


def _postprocess_action_chunk(postprocessor: Any, action_chunk: Any) -> torch.Tensor:
    action_tensor = action_chunk if torch.is_tensor(action_chunk) else torch.as_tensor(action_chunk)
    if action_tensor.ndim == 2:
        action_tensor = action_tensor.unsqueeze(0)
    if action_tensor.ndim != 3:
        raise ValueError(
            f"Expected action chunk with shape (B, T, D) or (T, D), got {tuple(action_tensor.shape)}"
        )

    processed_actions: list[torch.Tensor] = []
    for step_index in range(action_tensor.shape[1]):
        single_action = action_tensor[:, step_index, :]
        processed_action = postprocessor(single_action)
        if not torch.is_tensor(processed_action):
            processed_action = torch.as_tensor(processed_action)
        processed_actions.append(processed_action)

    stacked = torch.stack(processed_actions, dim=1)
    if stacked.shape[0] == 1:
        stacked = stacked.squeeze(0)
    return stacked.detach().to("cpu", dtype=torch.float32)


def _collect_gt_sequence(
    dataset: LeRobotDataset,
    start_index: int,
    episode_index: int,
    sequence_len: int,
    action_delta_indices: list[int] | None,
    gt_action_index: int | None,
) -> torch.Tensor | None:
    gt_actions: list[torch.Tensor] = []
    for offset in range(sequence_len):
        dataset_offset_index = start_index + offset
        if dataset_offset_index >= dataset.num_frames:
            return None

        sample = dataset[dataset_offset_index]
        future_episode_index = _scalar_to_int(sample.get("episode_index"), key="episode_index")
        if future_episode_index != episode_index:
            return None

        gt_actions.append(_extract_gt_action(sample[ACTION], action_delta_indices, gt_action_index).to("cpu"))

    return torch.stack(gt_actions, dim=0)


def _extract_observation_state(sample: dict[str, Any]) -> torch.Tensor | None:
    state = sample.get(OBS_STATE)
    if state is None:
        return None
    if torch.is_tensor(state):
        return state.detach().to("cpu", dtype=torch.float32)
    return torch.as_tensor(state, dtype=torch.float32)


def _reset_inference_state(*objects: Any) -> None:
    for obj in objects:
        reset_fn = getattr(obj, "reset", None)
        if callable(reset_fn):
            reset_fn()


def _load_dataset(cfg: OfflineDatasetPolicyEvalConfig) -> tuple[LeRobotDataset, dict[str, list[float]] | None]:
    dataset_cfg = cfg.dataset
    policy_cfg = cfg.policy
    assert policy_cfg is not None

    ds_meta = LeRobotDatasetMetadata(
        dataset_cfg.repo_id,
        root=dataset_cfg.root,
        revision=dataset_cfg.revision,
        force_cache_sync=dataset_cfg.force_cache_sync,
    )
    delta_timestamps = resolve_delta_timestamps(policy_cfg, ds_meta)
    selected_episodes = _resolve_selected_episodes(cfg)

    dataset = LeRobotDataset(
        dataset_cfg.repo_id,
        root=dataset_cfg.root,
        episodes=selected_episodes,
        delta_timestamps=delta_timestamps,
        tolerance_s=dataset_cfg.tolerance_s,
        revision=dataset_cfg.revision,
        force_cache_sync=dataset_cfg.force_cache_sync,
        download_videos=dataset_cfg.download_videos,
        video_backend=dataset_cfg.video_backend,
        return_uint8=dataset_cfg.return_uint8,
    )
    return dataset, delta_timestamps


def _make_processors(
    cfg: OfflineDatasetPolicyEvalConfig,
    dataset: LeRobotDataset,
) -> tuple[Any, Any]:
    policy_cfg = cfg.policy
    assert policy_cfg is not None

    dataset_stats = getattr(dataset.meta, "stats", None)
    preprocessor_overrides: dict[str, Any] = {
        "device_processor": {"device": policy_cfg.device},
        "rename_observations_processor": {"rename_map": cfg.rename_map},
    }
    postprocessor_overrides: dict[str, Any] = {}

    if dataset_stats:
        preprocessor_overrides["normalizer_processor"] = {
            "stats": dataset_stats,
            "features": {**policy_cfg.input_features, **policy_cfg.output_features},
            "norm_map": policy_cfg.normalization_mapping,
        }
        postprocessor_overrides["unnormalizer_processor"] = {
            "stats": dataset_stats,
            "features": policy_cfg.output_features,
            "norm_map": policy_cfg.normalization_mapping,
        }

    if getattr(policy_cfg, "use_relative_actions", False):
        preprocessor_overrides["relative_actions_processor"] = {
            "enabled": True,
            "exclude_joints": getattr(policy_cfg, "relative_exclude_joints", []),
            "action_names": getattr(policy_cfg, "action_feature_names", None),
        }
        postprocessor_overrides["absolute_actions_processor"] = {"enabled": True}

    processor_kwargs: dict[str, Any] = {
        "dataset_meta": dataset.meta,
        "preprocessor_overrides": preprocessor_overrides,
        "postprocessor_overrides": postprocessor_overrides,
    }
    if policy_cfg.pretrained_path is None and dataset_stats:
        processor_kwargs["dataset_stats"] = dataset_stats

    pretrained_path = str(policy_cfg.pretrained_path) if policy_cfg.pretrained_path is not None else None
    try:
        return make_pre_post_processors(
            policy_cfg,
            pretrained_path=pretrained_path,
            pretrained_revision=policy_cfg.pretrained_revision,
            **processor_kwargs,
        )
    except Exception as err:
        if pretrained_path is None:
            raise
        logging.warning(
            "Falling back to rebuilding processors from config because loading processors from '%s' failed: %s",
            pretrained_path,
            err,
        )
        processor_kwargs["dataset_stats"] = dataset_stats
        return make_pre_post_processors(policy_cfg, **processor_kwargs)


@parser.wrap()
def eval_offline_dataset_policy(cfg: OfflineDatasetPolicyEvalConfig) -> None:
    init_logging()
    logging.info("Configuration:\n%s", pformat(asdict(cfg)))

    dataset, delta_timestamps = _load_dataset(cfg)
    logging.info(
        "Dataset ready: repo_id=%s, episodes=%d, frames=%d, fps=%d, delta_timestamps=%s, selected_episodes=%s",
        dataset.repo_id,
        dataset.num_episodes,
        dataset.num_frames,
        dataset.fps,
        delta_timestamps,
        _resolve_selected_episodes(cfg),
    )

    policy_cfg = cfg.policy
    assert policy_cfg is not None
    policy = make_policy(cfg=policy_cfg, ds_meta=dataset.meta, rename_map=cfg.rename_map)
    policy.eval()

    preprocessor, postprocessor = _make_processors(cfg, dataset)

    total_frames = dataset.num_frames
    start_index = cfg.eval.start_index
    stop_index = total_frames if cfg.eval.stop_index is None else min(cfg.eval.stop_index, total_frames)
    if cfg.eval.max_samples is not None:
        stop_index = min(stop_index, start_index + cfg.eval.max_samples)

    if start_index >= total_frames:
        raise ValueError(
            f"start_index={start_index} is out of range for dataset with {total_frames} frame(s)."
        )
    if start_index >= stop_index:
        raise ValueError(
            f"Empty evaluation range after slicing: start_index={start_index}, stop_index={stop_index}."
        )

    csv_path = cfg.eval.csv_path.expanduser()
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    action_names = dataset.features.get(ACTION, {}).get("names")
    fieldnames = [
        "dataset_index",
        "episode_index",
        "frame_index",
        "task",
        "inference_mode",
        "effective_sequence_len",
        "l1",
        "l2",
        "max_abs",
        "observation_state",
        "gt_action",
        "pred_action",
        "abs_diff",
        "gt_action_sequence",
        "pred_action_sequence",
        "abs_diff_sequence",
    ]

    logging.info(
        "Evaluating dataset frame range [%d, %d) over %d selected frame(s). mode=%s sequence_len=%d",
        start_index,
        stop_index,
        stop_index - start_index,
        cfg.eval.inference_mode,
        cfg.eval.sequence_len,
    )

    current_episode: int | None = None
    warned_empty_task = False
    num_rows = 0
    skipped_short_horizon = 0
    l1_sum = 0.0
    l2_sum = 0.0
    max_abs_sum = 0.0
    per_dim_abs_sum: torch.Tensor | None = None

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        for dataset_index in tqdm(range(start_index, stop_index), desc="Offline policy eval"):
            sample = dataset[dataset_index]
            episode_index = _scalar_to_int(sample.get("episode_index"), key="episode_index")
            frame_index = _scalar_to_int(sample.get("frame_index"), key="frame_index")

            if current_episode != episode_index:
                _reset_inference_state(policy, preprocessor, postprocessor)
                current_episode = episode_index

            task = _resolve_task(sample, cfg.eval, dataset)
            if not task and not warned_empty_task:
                logging.warning(
                    "Some samples do not contain a task string. Using an empty string. "
                    "If the policy is language-conditioned, pass --eval.default_task=..."
                )
                warned_empty_task = True

            raw_observation = _batchify_observation(sample, task)
            processed_observation = preprocessor(raw_observation)
            observation_state = _extract_observation_state(sample)

            with torch.inference_mode():
                if cfg.eval.inference_mode == "select_action":
                    pred_action = policy.select_action(processed_observation)
                    pred_sequence = _ensure_action_sequence(postprocessor(pred_action)).to("cpu")
                else:
                    pred_chunk = policy.predict_action_chunk(processed_observation)
                    pred_sequence = _postprocess_action_chunk(postprocessor, pred_chunk)
                    if pred_sequence.shape[0] < cfg.eval.sequence_len:
                        raise ValueError(
                            "Predicted action chunk is shorter than requested sequence_len at "
                            f"dataset_index={dataset_index}: chunk_len={pred_sequence.shape[0]}, "
                            f"sequence_len={cfg.eval.sequence_len}"
                        )
                    pred_sequence = pred_sequence[: cfg.eval.sequence_len]

            gt_sequence = _collect_gt_sequence(
                dataset,
                dataset_index,
                episode_index,
                pred_sequence.shape[0],
                getattr(policy_cfg, "action_delta_indices", None),
                cfg.eval.gt_action_index,
            )
            if gt_sequence is None:
                skipped_short_horizon += 1
                continue

            if pred_sequence.shape != gt_sequence.shape:
                raise ValueError(
                    "Predicted action sequence shape does not match ground-truth sequence shape at "
                    f"dataset_index={dataset_index}: pred={tuple(pred_sequence.shape)}, "
                    f"gt={tuple(gt_sequence.shape)}"
                )

            diff_sequence = pred_sequence - gt_sequence
            abs_diff_sequence = diff_sequence.abs()
            l1 = float(abs_diff_sequence.mean().item())
            l2 = float(torch.linalg.vector_norm(diff_sequence.reshape(-1)).item())
            max_abs = float(abs_diff_sequence.max().item())

            gt_action_tensor = gt_sequence[0]
            pred_action_tensor = pred_sequence[0]
            abs_diff_tensor = abs_diff_sequence[0]
            per_dim_row_mae = abs_diff_sequence.mean(dim=0)

            writer.writerow(
                {
                    "dataset_index": dataset_index,
                    "episode_index": episode_index,
                    "frame_index": frame_index,
                    "task": task,
                    "inference_mode": cfg.eval.inference_mode,
                    "effective_sequence_len": int(pred_sequence.shape[0]),
                    "l1": l1,
                    "l2": l2,
                    "max_abs": max_abs,
                    "observation_state": json.dumps(
                        observation_state.tolist() if observation_state is not None else None,
                        ensure_ascii=False,
                    ),
                    "gt_action": json.dumps(gt_action_tensor.tolist(), ensure_ascii=False),
                    "pred_action": json.dumps(pred_action_tensor.tolist(), ensure_ascii=False),
                    "abs_diff": json.dumps(abs_diff_tensor.tolist(), ensure_ascii=False),
                    "gt_action_sequence": json.dumps(gt_sequence.tolist(), ensure_ascii=False),
                    "pred_action_sequence": json.dumps(pred_sequence.tolist(), ensure_ascii=False),
                    "abs_diff_sequence": json.dumps(abs_diff_sequence.tolist(), ensure_ascii=False),
                }
            )

            num_rows += 1
            l1_sum += l1
            l2_sum += l2
            max_abs_sum += max_abs
            per_dim_abs_sum = (
                per_dim_row_mae if per_dim_abs_sum is None else per_dim_abs_sum + per_dim_row_mae
            )

            if cfg.eval.save_every > 0 and num_rows % cfg.eval.save_every == 0:
                csv_file.flush()

            if cfg.eval.log_every > 0 and num_rows % cfg.eval.log_every == 0:
                logging.info(
                    "Processed %d rows | mean_l1=%.6f mean_l2=%.6f mean_max_abs=%.6f skipped_short_horizon=%d",
                    num_rows,
                    l1_sum / num_rows,
                    l2_sum / num_rows,
                    max_abs_sum / num_rows,
                    skipped_short_horizon,
                )

    if num_rows == 0:
        raise ValueError("No rows were evaluated.")

    logging.info("Saved frame-wise results to %s", csv_path.resolve())
    logging.info(
        "Summary | rows=%d mean_l1=%.6f mean_l2=%.6f mean_max_abs=%.6f skipped_short_horizon=%d",
        num_rows,
        l1_sum / num_rows,
        l2_sum / num_rows,
        max_abs_sum / num_rows,
        skipped_short_horizon,
    )

    if per_dim_abs_sum is not None and action_names and len(action_names) == per_dim_abs_sum.numel():
        per_dim_mae = per_dim_abs_sum / num_rows
        per_dim_summary = {
            str(name): round(float(value.item()), 6)
            for name, value in zip(action_names, per_dim_mae)
        }
        logging.info("Per-dimension MAE: %s", per_dim_summary)


def main() -> None:
    register_third_party_plugins()
    eval_offline_dataset_policy()


if __name__ == "__main__":
    main()
