#!/usr/bin/env python3
"""Small MJPEG camera preview server for Linux (V4L2)."""

from __future__ import annotations

import argparse
import html
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2


def parse_camera(path: str, default):
    query = parse_qs(urlparse(path).query)

    camera = query.get("camera", [default])[0]

    # 支持数字
    try:
        return int(camera)
    except ValueError:
        pass

    # 支持 /dev/video2
    return camera


class CameraHandler(BaseHTTPRequestHandler):
    default_camera = "/dev/video0"

    width = 640
    height = 480
    fps = 30
    jpeg_quality = 85

    def log_message(self, fmt, *args):
        print(f"{self.address_string()} - {fmt % args}")

    def do_GET(self):
        parsed = urlparse(self.path)

        camera = parse_camera(self.path, self.default_camera)

        if parsed.path in ("/", "/index.html"):
            self.serve_index(camera)

        elif parsed.path == "/stream.mjpg":
            self.serve_stream(camera)

        elif parsed.path == "/snapshot.jpg":
            self.serve_snapshot(camera)

        else:
            self.send_error(404)

    def serve_index(self, camera):

        links = []

        for i in range(12):
            dev = f"/dev/video{i}"

            active = camera == dev or camera == i

            links.append(
                f'<a class="{"active" if active else ""}" '
                f'href="/?camera={dev}">{dev}</a>'
            )

        body = f"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Linux Camera Preview</title>

<style>
body {{
background:#111;
color:#eee;
margin:0;
font-family:sans-serif;
}}

header {{
padding:10px;
background:#222;
}}

a {{
padding:6px 10px;
color:#9ecbff;
text-decoration:none;
}}

a.active {{
background:#3478f6;
color:white;
border-radius:4px;
}}

.wrap {{
display:grid;
place-items:center;
height:calc(100vh - 60px);
}}

img {{
max-width:100%;
max-height:100%;
}}
</style>

</head>

<body>

<header>
<strong>{html.escape(str(camera))}</strong>
{" ".join(links)}
</header>

<div class="wrap">
<img src="/stream.mjpg?camera={camera}">
</div>

</body>
</html>
"""

        body = body.encode()

        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def open_camera(self, camera):

        cap = cv2.VideoCapture(camera, cv2.CAP_V4L2)

        # print(self.width, self.height, self.fps)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        # 尝试使用 MJPEG，提高 USB 摄像头性能
        cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG"),
        )

        return cap

    def read_frame(self, cap):

        frame = None

        for _ in range(8):
            ok, img = cap.read()

            if ok:
                frame = img

            time.sleep(0.01)

        return frame

    def serve_snapshot(self, camera):

        cap = self.open_camera(camera)

        try:

            if not cap.isOpened():
                self.send_error(503, "cannot open camera")
                return

            frame = self.read_frame(cap)

            if frame is None:
                self.send_error(503, "no frame")
                return

            ok, jpg = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )

            if not ok:
                self.send_error(500)
                return

            data = jpg.tobytes()

            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()

            self.wfile.write(data)

        finally:
            cap.release()

    def serve_stream(self, camera):

        cap = self.open_camera(camera)

        if not cap.isOpened():
            self.send_error(503, "cannot open camera")
            return

        self.send_response(200)

        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )

        self.end_headers()

        try:

            while True:

                ok, frame = cap.read()

                if not ok:
                    continue

                ok, jpg = cv2.imencode(
                    ".jpg",
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )

                if not ok:
                    continue

                data = jpg.tobytes()

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type:image/jpeg\r\n")
                self.wfile.write(
                    f"Content-Length:{len(data)}\r\n\r\n".encode()
                )

                self.wfile.write(data)
                self.wfile.write(b"\r\n")

                time.sleep(1 / self.fps)

        except (BrokenPipeError, ConnectionResetError):
            pass

        finally:
            cap.release()


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)

    parser.add_argument(
        "--camera",
        default="/dev/video0",
        help="camera index or /dev/videoX",
    )

    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)

    args = parser.parse_args()

    CameraHandler.default_camera = args.camera
    CameraHandler.width = args.width
    CameraHandler.height = args.height
    CameraHandler.fps = args.fps

    server = ThreadingHTTPServer(
        (args.host, args.port),
        CameraHandler,
    )

    print(
        f"http://{args.host}:{args.port}/?camera={args.camera}"
    )

    server.serve_forever()


if __name__ == "__main__":
    main()