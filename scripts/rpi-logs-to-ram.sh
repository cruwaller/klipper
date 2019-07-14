#!/bin/bash
# This script locate critical logging folders into RAM to
# save SD card for writing

# Force script to exit if an error occurs
set -e

if [[ ${EUID} -ne 0 ]]; then
    echo "This script must run as root"
    exit -1
fi

add="$(grep LOGS_TO_RAM_START /etc/fstab)"
if [[ "${add}" == "" ]]; then
    /bin/sh -c "cat >> /etc/fstab" <<EOF

# LOGS_TO_RAM_START
tmpfs    /tmp        tmpfs    defaults,noatime,nosuid,size=100m    0 0
tmpfs    /var/tmp    tmpfs    defaults,noatime,nosuid,size=30m    0 0
tmpfs    /var/log    tmpfs    defaults,noatime,nosuid,mode=0755,size=100m    0 0
# LOGS_TO_RAM_END

EOF
    echo "  done."
else
    echo "  already done."
fi
echo ""
