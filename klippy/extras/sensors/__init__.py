# Printer heater support
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from thermistor import Thermistor, CustomThermistor
from linear import Linear, CustomLinear
from thermocouple import MAX31856, MAX31855, MAX6675
from rtd import MAX31865

Sensors = {
    "EPCOS 100K B57560G104F": {
        'class': Thermistor, 't1': 25., 'r1': 100000.,
        't2': 150., 'r2': 1641.9, 't3': 250., 'r3': 226.15},
    "ATC Semitec 104GT-2": {
        'class': Thermistor, 't1': 20., 'r1': 126800.,
        't2': 150., 'r2': 1360., 't3': 300., 'r3': 80.65},
    "NTC 100K beta 3950": {
        'class': Thermistor, 't1': 25., 'r1': 100000., 'beta': 3950.},
    "CustomThermistor": {
        'class': CustomThermistor},

    "AD595": {
        'class': Linear, 'values': [
            (0., .0027), (10., .101), (20., .200), (25., .250), (30., .300), (40., .401),
            (50., .503), (60., .605), (80., .810), (100., 1.015), (120., 1.219),
            (140., 1.420), (160., 1.620), (180., 1.817), (200., 2.015), (220., 2.213),
            (240., 2.413), (260., 2.614), (280., 2.817), (300., 3.022), (320., 3.227),
            (340., 3.434), (360., 3.641), (380., 3.849), (400., 4.057), (420., 4.266),
            (440., 4.476), (460., 4.686), (480., 4.896)
        ]},
    "PT100 INA826": {
        'class': Linear, 'values': [
            (0, 0.00), (1, 1.11), (10, 1.15), (20, 1.20), (30, 1.24), (40, 1.28),
            (50, 1.32), (60, 1.36), (70, 1.40), (80, 1.44), (90, 1.48), (100, 1.52),
            (110, 1.56), (120, 1.61), (130, 1.65), (140, 1.68), (150, 1.72), (160, 1.76),
            (170, 1.80), (180, 1.84), (190, 1.88), (200, 1.92), (210, 1.96), (220, 2.00),
            (230, 2.04), (240, 2.07), (250, 2.11), (260, 2.15), (270, 2.18), (280, 2.22),
            (290, 2.26), (300, 2.29), (310, 2.33), (320, 2.37), (330, 2.41), (340, 2.44),
            (350, 2.48), (360, 2.51), (370, 2.55), (380, 2.58), (390, 2.62), (400, 2.66),
            (500, 3.00), (600, 3.33), (700, 3.63), (800, 3.93), (900, 4.21),
            (1000, 4.48), (1100, 4.73)
        ]},
    "CustomLinear": {
        'class': CustomLinear},

    # Thermocouples readers (SPI)
    "MAX6675":  {'class': MAX6675},
    "MAX31855": {'class': MAX31855},
    "MAX31856": {'class': MAX31856},
    # RTD readers (SPI)
    "MAX31865": {'class': MAX31865},
}

def load_sensor(config):
    sensor_params = config.getchoice('sensor_type', Sensors)
    return sensor_params['class'](config, sensor_params)
