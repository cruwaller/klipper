#!/bin/bash
# This script enable RPi gpio serial and remove it from kernel

# Force script to exit if an error occurs
set -e

if [[ ${EUID} -ne 0 ]]; then
    echo "This script must run as root"
    exit -1
fi

if [[ ! -f /boot/cmdline.txt ]]; then
    echo "[ERROR] Can run only on RPi!"
    exit -1
fi

echo ""
echo "Disabling serial console..."

pi_version=0
serial="/dev/ttyAMA0"
if [[ -e /dev/ttyS0 && -e /dev/ttyAMA0 ]]; then
    # RPi 3 or newer
    pi_version=1
    serial="/dev/ttyS0"
fi

# Disabling the Console
sudo sed -i 's/ console=serial0,115200 / /g' /boot/cmdline.txt
if [[ $pi_version -eq 1 ]]; then
    sudo systemctl stop serial-getty@ttyS0.service
    sudo systemctl disable serial-getty@ttyS0.service
else
    sudo systemctl stop serial-getty@ttyAMA0.service
    sudo systemctl disable serial-getty@ttyAMA0.servicefi
fi

# Enable UART
add=$(grep enable_uart /boot/config.txt)
if [[ "${add}" == "" ]]; then
    /bin/sh -c "cat >> /boot/config.txt" <<EOF

# == Fix UART baudrate
core_freq=250
# == Enable uart ttyS0 (serial0)
enable_uart=1

EOF
fi

echo "  ...ready."
echo "  Now you can use RPi GPIO serial using ${serial}"
echo ""
