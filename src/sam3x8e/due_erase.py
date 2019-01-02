#!/usr/bin/env python

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    GPIO = None
import sys, os
import time

def init(reset, erase=None):
    if GPIO is None:
        return
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(reset, GPIO.OUT, initial=GPIO.HIGH)
    if erase is not None:
        GPIO.setup(erase, GPIO.OUT, initial=GPIO.LOW)

def make_reset(reset):
    # Reset logic is inverted, LOW == reset
    print("DUE reset... (pin %d)" % reset)
    if GPIO is None:
        return
    GPIO.output(reset, GPIO.LOW)
    time.sleep(.5)
    GPIO.output(reset, GPIO.HIGH)
    time.sleep(.2)

def make_erase(erase, reset):
    # Erase pin is not inverted, HIGH == erase
    print("DUE erase... (pin %d)" % erase)
    if GPIO is not None:
        GPIO.output(erase, GPIO.HIGH)
        time.sleep(1.)
        GPIO.output(erase, GPIO.LOW)
        time.sleep(.2)
    # Finally call reset to run into bootloader
    make_reset(reset)

def help():
    name = os.path.basename(sys.argv[0])
    print "Usage:"
    print "  Erase cmd:  python %s erase reset_pin erase_pin" % name
    print "  Reset cmd:  python %s reset reset_pin" % name
    exit(-1)

if __name__ == '__main__':
    try:
        mode = sys.argv[1]
        reset_pin = int(sys.argv[2])
        if mode == "erase":
            erase_pin = int(sys.argv[3])
            init(reset_pin, erase=erase_pin)
            make_erase(erase_pin, reset_pin)
        elif mode == "reset":
            init(reset_pin)
            make_reset(reset_pin)
        else:
            raise IndexError
    except IndexError:
        help()
