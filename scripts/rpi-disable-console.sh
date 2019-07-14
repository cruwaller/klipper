#!/bin/bash
# This script enable RPi gpio serial and remove it from kernel

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

version="$(cat /proc/device-tree/model |grep 'Pi 3')"
serial="ttyS0"
if [[ "${version}" == "" ]]; then
    serial="ttyAMA0"
fi
echo "    serial: ${serial}"

# Disabling the Console
sudo sed -i 's/ console=serial0,115200 / /g' /boot/cmdline.txt
sudo systemctl stop serial-getty@${serial}.service
sudo systemctl disable serial-getty@${serial}.service
echo "    console disabled"

# Enable UART
add="$(grep enable_uart /boot/config.txt)"
if [[ "${add}" == "" ]]; then
    /bin/sh -c "cat >> /boot/config.txt" <<EOF

# == Fix UART baudrate
core_freq=250
# == Enable uart ttyS0 (serial0)
enable_uart=1

EOF
fi
echo "    config.txt modified"
echo "  ...ready."
echo "  Now you can use RPi GPIO serial using /dev/${serial}"
echo ""
