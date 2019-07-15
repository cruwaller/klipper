#!/bin/bash
# This script installs Klipper on a Raspberry Pi machine running the
# debian buster distribution.
#
# Some preconditions are needed:
#   Install git
#     sudo apt-get install --yes git
#

MJPG_INSTALL_DIR="${HOME}/mjpg-streamer"

prepare_install()
{
    version=$(cat "/etc/debian_version")

    # Do only for debian 10.0 (buster)
    [ "${version}" != "10.0" ] && return

    [ -f "${HOME}/.klipper_prepare_done" ] && return

    report_status "Running apt-get clean..."
    sudo apt-get clean
    sudo rm -rf /var/lib/apt/lists/*
    sudo apt-get clean
    report_status "Running apt-get update..."
    sudo apt-get update
    report_status "Running apt-get upgrade..."
    sudo apt-get -y upgrade
    touch ${HOME}/.klipper_prepare_done
}

install_mjpg_streamer()
{
    if [ ! -d ${MJPG_INSTALL_DIR} ]; then
        report_status "Installing mjpg-streamer dependencies..."
        sudo apt-get install --yes cmake
        sudo apt-get install --yes libjpeg8-dev
        [ $? -ne 0 ] && sudo apt-get install --yes libjpeg62-turbo-dev
        report_status "Download mjpg-streamer..."
        git clone https://github.com/jacksonliam/mjpg-streamer.git ${MJPG_INSTALL_DIR}
    else
        git --git-dir=${MJPG_INSTALL_DIR}/.git pull
    fi
    report_status "Building mjpg-streamer..."
    make -C ${MJPG_INSTALL_DIR}/mjpg-streamer-experimental
}

install_mjpg_streamer_script()
{
    report_status "Installing mjpg-streamer system start script..."
    sudo cp "${SRCDIR}/scripts/webcamd-start.sh" /etc/init.d/webcamd
    sudo update-rc.d webcamd defaults
}

install_mjpg_streamer_defaults()
{
    DEFAULTS_FILE=/etc/default/webcamd
    [ -f $DEFAULTS_FILE ] && return

    report_status "Installing mjpg-streamer system start configuration..."
    sudo /bin/sh -c "cat > $DEFAULTS_FILE" <<EOF
# Configuration for /etc/init.d/webcamd

WEBCAM_ENABLED=1

WEBCAM_USER=$USER

WEBCAM_EXEC=${MJPG_INSTALL_DIR}/mjpg-streamer-experimental/mjpg_streamer

WEBCAM_PORT=8080

WEBCAM_DEV=/dev/video0

WEBCAM_RESOLUTION="640x480"

WEBCAM_FPS=1

EOF
}

start_mjpg_streamer()
{
    report_status "Launching webcamd service..."
    sudo /etc/init.d/webcamd restart
}

# Helper functions
report_status()
{
    echo -e "\n\n###### $1"
}

verify_ready()
{
    if [ "$EUID" -eq 0 ]; then
        echo "This script must not run as root"
        exit -1
    fi
}

# Force script to exit if an error occurs
set -e

# Find SRCDIR from the pathname of this script
SRCDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/.. && pwd )"

# Run installation steps defined above
verify_ready
prepare_install
install_mjpg_streamer
install_mjpg_streamer_script
install_mjpg_streamer_defaults
start_mjpg_streamer
${SRCDIR}/scripts/install-octopi.sh
