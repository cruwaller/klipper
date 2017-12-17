# system
import logging, types, struct
# project
from driverbase import DriverBase
import mcu, pins


class TMC2130(DriverBase):
    vsense = 0
    V_fs   = 0.325
    Rsense = None

    iHold      = 10
    iRun       = 21
    iHoldDelay = 8

    # Error flags
    isReset      = False
    isError      = False
    isStallguard = False
    isStandstill = False

    stealth_max_speed  = None
    coolstep_min_speed = 0
    diag0_stall = False
    diag1_stall = False

    def __init__(self, printer, config, logger):
        super(TMC2130, self).__init__(printer, config, None)
        self.name = config.section[7:]

        if logger is not None:
            self.logger = logger.getChild('tmc2130')
        else:
            self.logger = logging.getLogger("driver.%s"%(self.name,))

        self.current     = config.getfloat('current', 1000.0, above=200., maxval=1200.0)
        self.Rsense      = config.getfloat('sense_R', 0.11, above=0.09)
        self.hold_multip = config.getfloat('hold_multiplier', 0.5, above=0., maxval=1.0)
        self.interpolate = config.getboolean('interpolate', True)
        self.sensor_less_homing = \
            config.getboolean('sensor_less_homing', False)

        self.hybrid_threshold = config.getint('hybrid_threshold', None, minval=0, maxval=1048575)
        self.stealth_max_speed = config.getint('stealth_max_speed', None, minval=0)

        # +1...+63 = less sensitivity, -64...-1 = higher sensitivity
        self.sg_stall_value = config.getint('stall_threshold', 19, minval=-64, maxval=63)

        diag0types = {
            'NA'           : None,
            'errors'       : 0,
            'temp_prewarn' : 1,
            'stall'        : 2,
        }
        self.diag0purpose = config.getchoice('diag0_out', diag0types, default='NA')
        self.diag0act_high = config.getboolean('diag0_active_high', default=True)
        diag1types = {
            'NA'            : None,
            'stall'         : 0,
            'index'         : 1,
            'chopper_on'    : 2,
            'steps_skipped' : 3,
        }
        self.diag1purpose = config.getchoice('diag1_out', diag1types, default='NA')
        self.diag1act_high = config.getboolean('diag1_active_high', default=True)

        self.diag0_stall = True if self.diag0purpose is 2 else False
        self.diag1_stall = True if self.diag1purpose is 0 else False

        mSteps = { "256":256, "128":128, "64":64, "32":32,
                   "16":16, "8":8, "4":4, "2":2, "1":1 }
        self.microsteps = config.getchoice('microsteps', mSteps)

        mode = { "spreadCycle" : False, "stealthChop" : True }
        self.pwm_mode = config.getchoice('mode', mode, default='spreadCycle')

        if self.sensor_less_homing:
            self.coolstep_min_speed = 1024 * 1024 - 1
            self.pwm_mode = mode['spreadCycle']

        # ========== SPI config ==========
        spipin     = config.get('ss_pin')
        spimode    = config.getint('spi_mode', 3, minval=0, maxval=3)
        spispeed   = config.getint('spi_speed', 8000000)
        # setup SPI pins and configure mcu
        self.mcu_driver = pins.setup_pin(printer, 'spibus', spipin)
        self.mcu_driver.set_spi_settings(spimode, spispeed)
        self._mcu = self.mcu_driver.get_mcu()
        # self._mcu.add_config_object(self) # call build_config on connect
        self._mcu.register_init_cb(self.__init_callback)

        printer.add_object(self.name, self)
        self.logger.debug("config ok")

    '''
    ~                    READ / WRITE data transfer example
    =================================================================================
    action                       | data sent to TMC2130  | data received from TMC2130
    =================================================================================
    read DRV_STATUS              | --> 0x6F00000000      | <-- 0xSS & unused data
    read DRV_STATUS              | --> 0x6F00000000      | <-- 0xSS & DRV_STATUS
    write CHOPCONF := 0x00ABCDEF | --> 0xEC00ABCDEF      | <-- 0xSS & DRV_STATUS
    write CHOPCONF := 0x00123456 | --> 0xEC00123456      | <-- 0xSS00ABCDEF
    =================================================================================
    '''

    '''
    READ FRAME:
    |               40bit                        |
    | S 8bit |      32bit data                   |
    | STATUS |   D    |   D    |   D    |   D    |
    '''
    def __command_read(self, cmd, cnt=4): # 40bits always = 5 x 8bit!
        cmd &= 0x7F # Makesure the MSB is 0
        values = self.mcu_driver.read(cmd, cnt)
        # values = self.mcu_driver.read(cmd, cnt) # 2nd needed???
        # convert list of bytes to number
        val    = 0
        status = -1 # error
        size   = len(values)
        if 0 < size:
            status = values[0]
            if (status):
                if (status & 0x1):
                    self.isReset = True
                if (status & 0x2):
                    self.isError = True
                if (status & 0x4):
                    self.isStallguard = True
                if (status & 0x8):
                    self.isStandstill = True
            for idx in range(1, size):
                val <<= 8
                val |= values[idx]
        return val

    '''
    WRITE FRAME:
    |               40bit                        |
    | A 8bit |      32bit data                   |
    |  ADDR  |   D    |   D    |   D    |   D    |
    '''
    def __command_write(self, cmd, val=None): # 40bits always = 5 x 8bit!
        if val is not None:
            if type(val) is not types.ListType:
                #if val > 0xFFFFFFFF:
                #    conv = struct.Struct('>Q').pack
                #else:
                #    conv = struct.Struct('>I').pack
                conv = struct.Struct('>I').pack
                val = list(conv(val))
                #for idx in xrange(len(val)):
                #    try:
                #        val[idx] = int(val[idx])
                #    except:
                #        val[idx] = 0
            cmd |= 0x80
            self.mcu_driver.write(cmd, val)

    def __init_callback(self):
        #if self.diag0purpose = 0:


        self.set_GCONF(external_ref        = 1 if self.Rsense is None else 0,
                       internal_Rsense     = 1 if self.Rsense is not None else 0,
                       stealthChop         = self.pwm_mode,
                       diag0_errors        = 1 if self.diag0purpose is 0 else 0,
                       diag0_temp_prewarn  = 1 if self.diag0purpose is 1 else 0,
                       diag0_stall         = 1 if self.diag0purpose is 2 else 0,
                       diag1_stall         = 1 if self.diag1purpose is 0 else 0,
                       diag1_index         = 1 if self.diag1purpose is 1 else 0,
                       diag1_chopper_on    = 1 if self.diag1purpose is 2 else 0,
                       diag1_steps_skipped = 1 if self.diag1purpose is 3 else 0,
                       diag0_active_high   = self.diag0act_high,
                       diag1_active_high   = self.diag1act_high,)
        self.set_rms_current(self.current, self.Rsense, self.hold_multip)
        self.set_REG_TPOWERDOWN(128)
        if (self.pwm_mode is True):
            self.set_REG_PWMCONF()
            if (self.stealth_max_speed is not None):
                speed = 12650000 * self.microsteps / (self.stealth_max_speed * self.inv_step_dist * 256)
                self.set_REG_TPWMTHRS(speed)
            elif (self.hybrid_threshold is not None):
                self.set_REG_TPWMTHRS(self.hybrid_threshold)
        else:
            self.set_REG_TCOOLTHRS(self.coolstep_min_speed)
        if self.sensor_less_homing:
            self.set_REG_COOLCONF(sg_stall_value = self.sg_stall_value)
        # self.set_REG_THIGH()
        self.set_REG_CHOPCONF(vsense      = self.vsense,
                              interpolate = self.interpolate,
                              microsteps  = self.microsteps)
        # self.set_REG_ENCM_CTRL()

        self.get_REG_DRV_STATUS()

        self.logger.info(" init done!")

    #def build_config(self):
    #    self.logger.info("build_config()")




    #**************************************************************************
    # WRAPPER METHODS
    #**************************************************************************

    def clear_faults(self):
        self.isReset      = False
        self.isError      = False
        self.isStallguard = False
        self.isStandstill = False

    def set_rms_current(self,
                        current_in_mA  = 1000,
                        sense_R        = 0.11,
                        multip_for_holding_current = 0.5):
        '''
        '''
        CS = 32.0 * 1.41421 * current_in_mA / 1000.0 * (sense_R + 0.02) / 0.325 - 1

        # If Current Scale is too low, turn on high sensitivity R_sense and calculate again
        if (CS < 16):
            CS = 32.0 * 1.41421 * current_in_mA / 1000.0 * (sense_R + 0.02) / 0.180 - 1
            self.vsense = True
            self.V_fs   = 0.180
        else:
            # If CS >= 16, turn off high_sense_r if it's currently ON
            self.vsense = False
            self.V_fs   = 0.325

        self.set_IHOLD_IRUN( int(CS * multip_for_holding_current), int(CS), self.iHoldDelay);

        self.current     = current_in_mA
        self.Rsense      = sense_R
        self.hold_multip = multip_for_holding_current

    def get_rms_current(self):
        return ( self.iRun + 1 ) / 32.0 * self.V_fs / (self.Rsense + 0.02) / 1.41421 * 1000;



    #**************************************************************************
    # GENERAL CONFIGURATION REGISTERS (0x00..0x0F)
    #**************************************************************************
    def set_GCONF(self,
                  external_ref        = 0, # user internal by defaul
                  internal_Rsense     = 1,
                  stealthChop         = 0,
                  commutation         = 0,
                  shaft_dir           = 0, # motor direction
                  diag0_errors        = 1,
                  diag0_temp_prewarn  = 0,
                  diag0_stall         = 0,
                  diag1_stall         = 0,
                  diag1_index         = 0,
                  diag1_chopper_on    = 0,
                  diag1_steps_skipped = 0,
                  diag0_active_high   = 0,
                  diag1_active_high   = 0,
                  small_hysterisis    = 0,
                  stop_enable         = 0,
                  direct_mode         = 0):
        '''
        external_ref        0/1    Use external voltage reference for coil currents
        internal_Rsense     0/1    Use internal sense resistors
        stealthChop         0/1    Enable stealthChop (dependant on velocity thresholds)
        commutation         0/1    Enable commutation by full step encoder
        shaft_dir           0/1    Inverse motor direction
        diag0_errors        0/1    Enable DIAG0 active on driver errors:
                                       Over temperature (ot),
                                       short to GND (s2g),
                                       undervoltage chargepump (uv_cp)
        diag0_temp_prewarn  0/1    Enable DIAG0 active on driver over temperature prewarning
        diag0_stall         0/1    Enable DIAG0 active on motor stall (set TCOOLTHRS before using this feature)
        diag1_stall         0/1    Enable DIAG1 active on motor stall (set TCOOLTHRS before using this feature)
        diag1_index         0/1    Enable DIAG1 active on index position (microstep look up table position 0)
        diag1_chopper_on    0/1    Enable DIAG1 active when chopper is on
        diag1_steps_skipped 0/1    Enable output toggle when steps are skipped in dcStep mode (increment of LOST_STEPS).
                                   Do not enable in conjunction with other DIAG1 options.
        diag0_active_high   0/1    0: DIAG0 is open collector output (active low)
                                   1: Enable DIAG0 push pull output (active high)
        diag1_active_high   0/1    0: DIAG1 is open collector output (active low)
                                   1: Enable DIAG1 push pull output (active high)
        small_hysterisis    0/1    0: Hysteresis for step frequency comparison is 1/16
                                   1: Hysteresis for step frequency comparison is 1/32
        stop_enable         0/1    Emergency stop: DCIN stops the sequencer when tied high
                                   (no steps become executed by the sequencer, motor goes to standstill state).
        direct_mode         0/1    Motor coil currents and polarity are directly controlled by the SPI interface.
        '''
        commutation = 0
        direct_mode = 0

        reg = 0x00
        val = 0
        # 17 = test_mode , should be set to 0
        val = ((val << 1) | (direct_mode         & 1)) # 16
        val = ((val << 1) | (stop_enable         & 1)) # 15
        val = ((val << 1) | (small_hysterisis    & 1)) # 14
        val = ((val << 1) | (diag1_active_high   & 1)) # 13
        val = ((val << 1) | (diag0_active_high   & 1)) # 12
        val = ((val << 1) | (diag1_steps_skipped & 1)) # 11
        val = ((val << 1) | (diag1_chopper_on    & 1)) # 10
        val = ((val << 1) | (diag1_index         & 1)) # 9
        val = ((val << 1) | (diag1_stall         & 1)) # 8
        val = ((val << 1) | (diag0_stall         & 1)) # 7
        val = ((val << 1) | (diag0_temp_prewarn  & 1)) # 6
        val = ((val << 1) | (diag0_errors        & 1)) # 5
        val = ((val << 1) | (shaft_dir           & 1)) # 4
        val = ((val << 1) | (commutation         & 1)) # 3
        val = ((val << 1) | (stealthChop         & 1)) # 2
        val = ((val << 1) | (internal_Rsense     & 1)) # 1
        val = ((val << 1) | (external_ref        & 1)) # 0
        self.__command_write(reg, val)

    def get_GSTAT(self):
        # R+C
        return self.__command_read(0x01)


    #**************************************************************************
    # VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10..0x1F)
    #**************************************************************************
    def set_IHOLD_IRUN(self,
                       hold_current = 10,
                       run_current  = 20,
                       hold_delay   = 8):

        '''
        hold_current 0..31   Standstill current (0=1/32...31=32/32)
        run_current  0..31   Motor run current (0=1/32...31=32/32)
        hold_delay   0..15   Controls the number of clock cycles for motor power down after a
                             motion as soon as standstill is detected (stst=1) and TPOWERDOWN has expired.
        '''
        reg = 0x10
        val = 4 * [0]
        val[3] = hold_current & 0x1F  # IHOLD      bits  4.. 0
        val[2] = run_current  & 0x1F  # IRUN       bits 12.. 8
        val[1] = hold_delay   & 0x0F  # IHOLDDELAY bits 19..16
        self.__command_write(reg, val)

        self.iHold      = val[3]
        self.iRun       = val[2]
        self.iHoldDelay = val[1]

    def set_REG_TPOWERDOWN(self, power_down_delay):
        '''
        Range 0...255
        power_down_delay sets the delay time after stand still (stst) of the motor
        to motor current power down. Time range is about 0 to 4 seconds.
        delay: 0...((2^8)-1) * 2^18 tCLK
        '''
        reg = 0x10
        val = power_down_delay & 0xFF
        self.__command_write(reg, val)

    def get_REG_TSTEP(self, microstep_time):
        '''
        Read the actual measured time between two 1/256 microsteps
        derived from the step input frequency in units of 1/fCLK.

        microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP
        '''
        return self.__command_read(0x12)

    def set_REG_TPWMTHRS(self, stealthChop_max_speed):
        '''
        0..1,048,575
        This is the upper velocity for stealthChop voltage PWM mode.
        TSTEP >= TPWMTHRS:
              - stealthChop PWM mode is enabledif configured
              - dcStep is disabled
        '''
        reg = 0x13
        val = stealthChop_max_speed & 0xFFFFF
        self.__command_write(reg, val)

    def set_REG_TCOOLTHRS(self, coolstep_min_speed):
        '''
        0..1,048,575
        This is the lower threshold velocity for switching on smart
        energy coolStep and stallGuard feature

        Set this parameter to disable coolStep at low speeds, where it
        cannot work reliably. The stall detection and stallGuard output
        signal becomes enabled when exceeding this velocity. In nondcStep
        mode, it becomes disabled again once the velocity falls
        below this threshold.

        TCOOLTHRS >= TSTEP >= THIGH:
          - coolStep is enabled, if configured
          - stealthChop voltage PWM mode is disabled
        TCOOLTHRS >= TSTEP
          - Stop on stall and stall output signal is enabled, if configured
        '''
        reg = 0x14
        val = coolstep_min_speed & 0xFFFFF
        self.__command_write(reg, val)

    def set_REG_THIGH(self, mode_sw_speed):
        '''
        0..1,048,575
        This velocity setting allows velocity dependent switching into a
        different chopper mode and fullstepping to maximize torque.

        The stall detection feature becomes switched off for 2-3
        electrical periods whenever passing THIGH threshold to
        compensate for the effect of switching modes.

        TSTEP <= THIGH:
          - coolStep is disabled (motor runs with normal current scale)
          - stealthChop voltage PWM mode is disabled
          - If vhighchm is set, the chopper switches to chm=1
            with TFD=0 (constant off time with slow decay, only).
          - chopSync2 is switched off (SYNC=0)
          - If vhighfs is set, the motor operates in fullstep mode
            and the stall detection becomes switched over to
            dcStep stall detection.
        '''
        reg = 0x15
        val = mode_sw_speed & 0xFFFFF
        self.__command_write(reg, val)


    #**************************************************************************
    # SPI MODE REGISTER SET
    #**************************************************************************
    def set_REG_XDRIRECT(self, coil_A_current, coil_B_current):
        '''
        255..+255
        Specifies Motor coil currents and polarity directly
        programmed via the serial interface. In this mode,
        the current is scaled by IHOLD setting.
        '''
        reg = 0x2D
        # SKIP!
        pass

    #**************************************************************************
    # dcStep Minimum Velocity Register
    #**************************************************************************
    def set_REG_VDCMIN(self, dcStep_min_speed):
        '''
        0..8,388,607
        The automatic commutation dcStep becomes enabled by the external signal DCEN.
        VDCMIN is used as the minimum step velocity when the motor is heavily loaded.
        Hint: Also set DCCTRL parameters in order to operate dcStep.

        time reference t for VDCMIN: t = 2^24 / fCLK
        '''
        reg = 0x33
        pass

    #**************************************************************************
    # MICROSTEPPING CONTROL REGISTER SET (0x60..0x6B)
    #**************************************************************************


    #**************************************************************************
    # DRIVER REGISTER SET (0X6C...0X7F)
    #**************************************************************************

    def set_REG_CHOPCONF(self,
                         off_time                 = 5, # Only enables the driver if used with stealthChop
                         hysterisis_start         = 0,
                         fast_decay_time          = 8,
                         hysterisis_end           = 1,
                         sine_offset              = 0,
                         disable_I_comparator     = 0,
                         random_off_time          = 1,
                         chopper_mode             = 1,
                         blank_time               = 36,
                         vsense                   = 0,
                         fullstep_threshold       = 0,
                         high_speed_mode          = 1,
                         sync_phases              = 8,
                         microsteps               = 32,
                         interpolate              = 1,
                         double_edge_step         = 1,
                         disable_short_protection = 0):
        '''
        off_time,                  0..15              Off time setting controls duration of slow decay phase
                                                      NCLK=12 + 32*TOFF Initialized to value 2 (NCLK = 76) by begin()
        hysterisis_start,          1..8               Add 1, 2, ..., 8 to hysteresis low value HEND (1/512 of this
                                                      setting adds to current setting)
                                                      Attention: Effective HEND+HSTRT <= 16.
                                                      Hint: Hysteresis decrement is done each 16 clocks
        fast_decay_time,           0..15              Fast decay time setting TFD with NCLK= 32*HSTRT
        hysterisis_end,            -3..12             This is the hysteresis value which becomes used for the hysteresis chopper.
        sine_offset,               -3..12             This is the sine wave offset and 1/512 of the value becomes
                                                      added to the absolute value of each sine wave entry.
        disable_I_comparator,      0/1                1: Disables current comparator usage for termination of the
                                                         fast decay cycle. chopper_mode needs to be 1.
        random_off_time,           0/1                0: Chopper off time is fixed as set by TOFF
                                                      1: Random mode, TOFF is random modulated by dNCLK= -12 ... +3 clocks.
        chopper_mode,              0/1                0: Standard mode (spreadCycle)
                                                      1: Constant off time with fast decay time.
                                                      Fast decay time is also terminated when the negative nominal
                                                      current is reached. Fast decay is after on time.
        blank_time,                16, 24, 36, 54     Set comparator blank time to 16, 24, 36 or 54 clocks.
                                                      Hint: 24 or 36 is recommended for most applications initialized
                                                            to 36 (register value = 3) by begin()
        vsense,                    0/1                0: Low sensitivity, high sense resistor voltage
                                                      1: High sensitivity, low sense resistor voltage
        fullstep_threshold,        0/1                This bit enables switching to fullstep, when VHIGH is exceeded.
                                                      Switching takes place only at 45deg position. The fullstep target
                                                      current uses the current value from the microstep table at
                                                      the 45deg position.
        high_speed_mode,           0/1                This bit enables switching to chm=1 and fd=0, when VHIGH is exceeded.
                                                      This way, a higher velocity can be achieved. Can be combined with
                                                      vhighfs=1. If set, the TOFF setting automatically becomes doubled
                                                      during high velocity operation in order to avoid doubling of the
                                                      chopper frequency.
        sync_phases,               0..15              Synchronization of the chopper for both phases of a two phase motor
                                                      in order to avoid the occurrence of a beat, especially at low motor
                                                      velocities. It is automatically switched off above VHIGH.
        microsteps,                255, 128, 64, 32,  Reduced microstep resolution for Step/Dir operation.
                                   16, 8, 4, 2,         The resolution gives the number of microstep entries
                                   0 (FULLSTEP)         per sine quarter wave.
        interpolate,               0/1                The actual microstep resolution becomes extrapolated
                                                      to 256 microsteps for smoothest motor operation.
        double_edge_step,          0/1                Enable step impulse at each step edge to reduce step frequency requirement.
        disable_short_protection,  0/1                0: Short to GND protection is on
                                                      1: Short to GND protection is disabled
        '''

        msteps_map = {
            256 : 0b0000, # 0 : 256x microstepping (Native)
            128 : 0b0001, # 1 : 128x microstepping
            64  : 0b0010, # 2 :  64x microstepping
            32  : 0b0011, # 3 :  32x microstepping
            16  : 0b0100, # 4 :  16x microstepping
            8   : 0b0101, # 5 :   8x microstepping
            4   : 0b0110, # 6 :   4x microstepping
            2   : 0b0111, # 7 :   2x microstepping
            1   : 0b1000  # 8 :   1x full-step
        }
        blank_time_map = {
            16 : 0b00,
            24 : 0b01,    # Recommended for most applications.
            36 : 0b10,    # Recommended for most applications.
            54 : 0b11
            }

        reg = 0x6C
        val = 0
        val = ((val << 1) | (disable_short_protection & 0x1)) # bit  30
        val = ((val << 1) | (double_edge_step         & 0x1)) # bit  29
        val = ((val << 1) | (interpolate              & 0x1)) # bit  28
        val = ((val << 4) | msteps_map[microsteps])           # bits 27...24
        val = ((val << 4) | (sync_phases              & 0xF)) # bits 23...20
        val = ((val << 1) | (high_speed_mode          & 0x1)) # bit  19
        val = ((val << 1) | (fullstep_threshold       & 0x1)) # bit  18
        val = ((val << 1) | (vsense                   & 0x1)) # bit  17
        val = ((val << 2) | blank_time_map[blank_time])       # bits 16...15
        val = ((val << 1) | (chopper_mode             & 0x1)) # bit  14
        val = ((val << 1) | (random_off_time          & 0x1)) # bit  13
        val = ((val << 1) | (disable_I_comparator     & 0x1)) # bit  12
        if (chopper_mode):
            val = ((val << 1) | (fast_decay_time      & 0x8)) # bit  11 - MSB of the fast_decay_time
            val = ((val << 4) | (sine_offset          & 0xF)) # bits 10...7
            val = ((val << 3) | (fast_decay_time      & 0x7)) # bits  6...4
        else:
            val <<= 1                                         # bit  11 - reserverx
            val = ((val << 4) | (hysterisis_end       & 0xF)) # bits 10...7
            val = ((val << 3) | (hysterisis_start     & 0x7)) # bits  6...4
        val = ((val << 4) | (off_time                 & 0xF)) # bits  3...0
        self.__command_write(reg, val)

    def set_REG_COOLCONF(self,
                         sg_min              = 0,
                         sg_max              = 0,
                         sg_step_width       = 1,
                         sg_current_decrease = 32,
                         smart_min_current   = 0,
                         sg_stall_value      = 19,
                         sg_filter           = 0):
        '''
        sg_min               0..15       If the stallGuard2 result falls below sg_min*32, the motor current
                                         becomes increased to reduce motor load angle.
        sg_max               0..15       If the stallGuard2 result is equal to or above (sg_min+sg_max+1)*32,
                                         the motor current becomes decreased to save energy.
        sg_step_width        1, 2, 4, 8  Current increment steps per measured stallGuard2 value
        sg_current_decrease  1, 2, 8, 32 For each (value) stallGuard2 values decrease by one
        smart_min_current    0/1         0: 1/2 of current setting (IRUN)
                                         1: 1/4 of current setting (IRUN)
        sg_stall_value       int         This signed value controls stallGuard2 level for stall output and
                                         sets the optimum measurement range for readout. A lower value gives
                                         a higher sensitivity. Zero is the starting value working with most motors.
                                         -64 to +63: A higher value makes stallGuard2 less sensitive and requires
                                         more torque to indicate a stall.
        sg_filter            0/1         0: Standard mode, high time resolution for stallGuard2
                                         1: Filtered mode, stallGuard2 signal updated for each four fullsteps
                                            (resp. six fullsteps for 3 phase motor) only to compensate
                                            for motor pole tolerances
        '''
        seup_t = {
            1 : 0b00, # Current increment steps per measured stallGuard2 value = 1
            2 : 0b01, # Current increment steps per measured stallGuard2 value = 2
            4 : 0b10, # Current increment steps per measured stallGuard2 value = 4
            8 : 0b11  # Current increment steps per measured stallGuard2 value = 8
        }
        sedn_t = {
            32 : 0b00, # %00: For each 32 stallGuard2 values decrease by one
            8  : 0b01, # %01: For each 8  stallGuard2 values decrease by one
            2  : 0b10, # %10: For each 2  stallGuard2 values decrease by one
            1  : 0b11  # %11: For each    stallGuard2 value  decrease by one
        }
        reg = 0x6D
        val = 0
        val = ((val << 1) | (sg_filter & 0x1))             # bit  24
        val <<= 1                                          # bit  23 reserved
        val = ((val << 7) | (sg_stall_value & 0x7F))       # bits 22...16 (7)
        val = ((val << 1) | (smart_min_current & 0x1))     # bit  15 (1)
        val = ((val << 2) | (sedn_t[sg_current_decrease])) # bits 14..13 (2)
        val << 1                                           # bit  12 reserved
        val = ((val << 4) | (sg_max & 0xF))                # bits 11...8 (4)
        val << 1                                           # bit  7 reserved
        val = ((val << 2) | (seup_t[sg_step_width]))       # bits 6...5 (2)
        val <<= 1                                          # bit  4 reserved
        val = ((val << 4) | (sg_min & 0xF))                # bits 3...0 (4)
        self.__command_write(reg, val)

    def get_REG_DRV_STATUS(self, ):
        val = self.__command_read(0x6F)
        self.logger.debug("status ( reg value = {} )".format(hex(val)))

        if (val & 0b10000000000000000000000000000000):
            # 31: standstill indicator
            self.logger.info("Stand still")
        if (val & 0b00010000000000000000000000000000):
            # 28: short to ground indicator phase B
            self.logger.error("Phase B short to ground")
        if (val & 0b00001000000000000000000000000000):
            # 27: short to ground indicator phase A
            self.logger.error("Phase A short to ground")
        if (val & 0b00000100000000000000000000000000):
            # 26: overtemperature prewarning
            self.logger.warning("Over temperature prewarning!")
        if (val & 0b00000010000000000000000000000000):
            # 25: overtemperature
            self.logger.error("Over temperature!")
        if (val & 0b00000001000000000000000000000000):
            # 24: stallGuard2 status
            self.logger.error("motor stall")

        # 20-16: actual motor current / smart energy current
        CS_ACTUAL = (val & 0b00000000000111110000000000000000) >> 16
        # Stall Guard status
        SG_RESULT = (val & 0b00000000000000000000001111111111)

        self.logger.debug("DRV_STATUS : CS Actual = %d, SG Result = %d" %
                          (CS_ACTUAL, SG_RESULT))
        return val

    def set_REG_PWMCONF(self,
                        stealth_amplitude = 255,
                        stealth_gradient  = 5,
                        stealth_freq      = "fPWM_2/683",
                        stealth_autoscale = 1,
                        stealth_symmetric = 0,
                        standstill_mode   = "FREEWHEEL_NORMAL"):
        # reset default 0x00050480
        '''
        stealth_amplitude     0..255  pwm_autoscale = 0
                                        User defined PWM amplitude offset (0-255)
                                        The resulting amplitude (limited to 0...255) is:
                                        PWM_AMPL + PWM_GRAD * 256 / TSTEP
                                      pwm_autoscale = 1
                                        User defined maximum PWM amplitude when switching back
                                        from current chopper mode to voltage PWM mode
                                        (switch over velocity defined by TPWMTHRS).
                                        Do not set too low values, as the regulation cannot
                                        measure the current when the actual PWM value goes below
                                        a setting specific value. Settings above 0x40 recommended.

        stealth_gradient      0..255  pwm_autoscale = 0
                                        Velocity dependent gradient for PWM amplitude:
                                        PWM_GRAD * 256 / TSTEP is added to PWM_AMPL
                                      pwm_autoscale = 1
                                        User defined maximum PWM amplitude change per
                                        half wave (1 to 15)

        stealth_freq          0..3    0: fPWM=2/1024 fCLK
                                      1: fPWM=2/683 fCLK
                                      2: fPWM=2/512 fCLK
                                      3: fPWM=2/410 fCLK
        stealth_autoscale     0/1     0: User defined PWM amplitude. The current settings have no influence.
                                      1: Enable automatic current control Attention: When using a user
                                         defined sine wave table, the amplitude of this sine wave table
                                         should not be less than 244. Best results are obtained
                                         with 247 to 252 as peak values.
        stealth_symmetric     0/1     0: The PWM value may change within each PWM cycle (standard mode)
                                      1: A symmetric PWM cycle is enforced
        standstill_mode       0..3    Stand still option when motor current setting is zero (I_HOLD=0).
                                      0: Normal operation
                                      1: Freewheeling
                                      2: Coil shorted using LS drivers
                                      3: Coil shorted using HS drivers
        '''
        pwm_freq_t = {
            "fPWM_2/1024" : 0b00, # %00: fPWM = 2/1024 fCLK
            "fPWM_2/683"  : 0b01, # %01: fPWM = 2/683  fCLK
            "fPWM_2/512"  : 0b10, # %10: fPWM = 2/512  fCLK
            "fPWM_2/410"  : 0b11  # %11: fPWM = 2/410  fCLK
        }
        freewheel_t = {
            "FREEWHEEL_NORMAL"    : 0b00, # %00: Normal operation
            "FREEWHEEL_FREEWHEEL" : 0b01, # %01: Freewheeling
            "FREEWHEEL_SHORT_LS"  : 0b10, # %10: Coil shorted using LS drivers
            "FREEWHEEL_SHORT_HS"  : 0b11  # %11: Coil shorted using HS drivers
        }
        reg = 0x70
        val = 0
        val = ((val << 2) | freewheel_t[standstill_mode]) # bits 21..20
        val = ((val << 1) | (stealth_symmetric & 1))      # bit  19
        val = ((val << 1) | (stealth_autoscale & 1))      # bit  18
        val = ((val << 2) | pwm_freq_t[stealth_freq])     # bits 17..16
        val = ((val << 8) | (stealth_gradient & 0xFF))    # bits 15...8
        val = ((val << 8) | (stealth_amplitude & 0xFF))   # bits 7...0
        self.__command_write(reg, val)

    def set_REG_ENCM_CTRL(self,
                          invert_encoder = 0,
                          maxspeed       = 0):
        '''
        invert_encoder     0/1    Invert encoder inputs
        maxspeed           0/1    Ignore Step input. If set, the hold current IHOLD determines
                                  the motor current, unless a step source is activated
        '''
        reg  = 0x72
        val  = (maxspeed       & 1) << 1
        val |= (invert_encoder & 1)
        self.__command_write(reg, val)

    def get_LOST_STEPS(self):
        val = self.__command_read(0x73)
        if val:
            self.logger.error("TMC2130 - LOST_STEPS: {}".format(hex(val)))
        return val
