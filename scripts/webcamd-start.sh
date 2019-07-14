#!/bin/sh
# System startup script for mjpg_streamer

### BEGIN INIT INFO
# Provides:          webcamd
# Required-Start:    $local_fs networking
# Required-Stop:
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: webcam daemon
# Description:       Starts mjpg_streamer daemon.
### END INIT INFO

PATH=/usr/local/sbin:/usr/local/bin:/sbin:/bin:/usr/sbin:/usr/bin
DESC="Webcam daemon"
NAME="webcamd"
DEFAULTS_FILE=/etc/default/webcamd
PIDFILE=/var/run/webcamd.pid

. /lib/lsb/init-functions

# Read defaults file
[ -r $DEFAULTS_FILE ] && . $DEFAULTS_FILE

if [ -z "${WEBCAM_ENABLED}" -o "${WEBCAMENABLED}" != "1" ]; then
   log_warning_msg "Not starting $NAME, disabled in $DEFAULTS_FILE."
   exit 0
fi

WEBCAM_ARGS='-i "input_uvc.so -n -r ${WEBCAM_RESOLUTION} -f ${WEBCAM_FPS} -d ${WEBCAM_DEV}" -o "output_http.so -p ${WEBCAM_PORT}"'

case "$1" in
start)  log_daemon_msg "Starting" $NAME
        start-stop-daemon --start --quiet --exec $WEBCAM_EXEC \
                          --background --pidfile $PIDFILE --make-pidfile \
                          --chuid $WEBCAM_USER --user $WEBCAM_USER \
                          -- $WEBCAM_ARGS
        log_end_msg $?
        ;;
stop)   log_daemon_msg "Stopping" $NAME
        killproc -p $PIDFILE $WEBCAM_EXEC
        RETVAL=$?
        [ $RETVAL -eq 0 ] && [ -e "$PIDFILE" ] && rm -f $PIDFILE
        log_end_msg $RETVAL
        ;;
restart) log_daemon_msg "Restarting" $NAME
        $0 stop
        $0 start
        ;;
reload|force-reload)
        log_daemon_msg "Reloading configuration not supported" $NAME
        log_end_msg 1
        ;;
status)
        status_of_proc -p $PIDFILE $WEBCAM_EXEC $NAME && exit 0 || exit $?
        ;;
*)      log_action_msg "Usage: /etc/init.d/webcamd {start|stop|status|restart|reload|force-reload}"
        exit 2
        ;;
esac
exit 0
