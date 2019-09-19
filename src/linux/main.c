// Main starting point for micro-controller code running on linux systems
//
// Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"
#include </usr/include/sched.h> // sched_setscheduler
#include <stdio.h> // fprintf
#include <string.h> // memset
#include <unistd.h> // getopt
#include <ctype.h>  // isprint
#include "board/misc.h" // console_sendf
#include "command.h" // DECL_CONSTANT
#include "internal.h" // console_setup
#include "sched.h" // sched_main

#if (CONFIG_SIMULATOR == 1)
DECL_CONSTANT_STR("MCU", CONFIG_MCU);
int SIMULATOR_MODE = CONFIG_SIMULATOR;
#else
DECL_CONSTANT_STR("MCU", "linux");
#endif


/****************************************************************
 * Real-time setup
 ****************************************************************/

static int
realtime_setup(void)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = 1;
    int ret = sched_setscheduler(0, SCHED_FIFO, &sp);
    if (ret < 0) {
        report_errno("sched_setscheduler", ret);
        return -1;
    }
    return 0;
}


/****************************************************************
 * Restart
 ****************************************************************/

static char **orig_argv;

void
command_config_reset(uint32_t *args)
{
    (void)args;
    if (! sched_is_shutdown())
        shutdown("config_reset only available when shutdown");
    int ret = execv(orig_argv[0], orig_argv);
    report_errno("execv", ret);
}
DECL_COMMAND_FLAGS(command_config_reset, HF_IN_SHUTDOWN, "config_reset");


/****************************************************************
 * Startup
 ****************************************************************/

int
main(int argc, char **argv)
{
    char * ttystr = "/tmp/klipper_host_mcu";

    // Parse program args
    orig_argv = argv;
    int opt, watchdog = 0, realtime = 0;
    while ((opt = getopt(argc, argv, "wrt:")) != -1) {
        switch (opt) {
        case 'w':
            watchdog = 1;
            break;
        case 'r':
            realtime = 1;
            break;
        case 't':
            ttystr = optarg;
            break;
        default:
            if (optopt == 't')
                fprintf (stderr, "Option -%c requires an argument.\n", optopt);
            else if (isprint (optopt))
                fprintf (stderr, "Unknown option `-%c'.\n", optopt);
            else
                fprintf(stderr, "Usage: %s [-w] [-r]\n", argv[0]);
            return -1;
        }
    }

    // Initial setup
    if (realtime) {
        int ret = realtime_setup();
        if (ret)
            return ret;
    }
#if (CONFIG_SIMULATOR == 1)
    printf("Init TTY: %s\n", ttystr);
#endif
    int ret = console_setup(ttystr);
    if (ret)
        return -1;
#if (CONFIG_SIMULATOR == 1)
    printf("TTY ready\n");
#endif
    if (watchdog) {
        int ret = watchdog_setup();
        if (ret)
            return ret;
    }

    // Main loop
    sched_main();
    return 0;
}
