#!/bin/bash

# Define files to copy
UART_SN_RULES="99-serial-devices.rules"
CAM_SN_RULES="99-usb-cameras.rules"

# Copy file if it exists
if [ -f "$UART_SN_RULES" ]; then
    echo "Copying $UART_SN_RULES to /etc/udev/rules.d..."
    chmod +x "$UART_SN_RULES"
    sudo cp "$UART_SN_RULES" /etc/udev/rules.d
    if [ $? -eq 0 ]; then
        echo "Success: $UART_SN_RULES copied to /etc/udev/rules.d"
    else
        echo "Error: Failed to copy $UART_SN_RULES"
        exit 1
    fi
fi

if [ -f "$CAM_SN_RULES" ]; then
    echo "Copying $CAM_SN_RULES to /etc/udev/rules.d..."
    chmod +x "$CAM_SN_RULES"
    sudo cp "$CAM_SN_RULES" /etc/udev/rules.d
    if [ $? -eq 0 ]; then
        echo "Success: $CAM_SN_RULES copied to /etc/udev/rules.d"
    else
        echo "Error: Failed to copy $CAM_SN_RULES"
        exit 1
    fi
fi
