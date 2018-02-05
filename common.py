#!/usr/bin/python

import math

########################################################################################################################
# MATH / COMPUTATIONAL FUNCTIONS
########################################################################################################################

def twos_compliment(val):
    if val >= 0x8000:
        return-((65535 - val) + 1)
    else:
        return val

# Pythagorean theorem of two lengths
def dist(a, b):

    return math.sqrt((a * a) + (b * b))

def get_y_rotation(x, y, z):

    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)

def get_x_rotation(x, y, z):

    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)

def get_z_rotation(x,y,z):

    radians = math.atan2(z, dist(x,y))
    return math.degrees(radians)

def get_CompassBearing(x, y):

    bearing = math.atan2(y, x)
    if bearing < 0:
        bearing += 2 * math.pi
    return math.degrees(bearing)

########################################################################################################################
# SENSOR COMMUNICATION [I2C]
########################################################################################################################

# Reads data from sensor's registers
def read_byte(bus, adr_device, adr_register):

    try:
       return bus.read_byte_data(adr_device, adr_register)

    except IOError as err:
        print(err)

def read_word(bus, adr_device, adr_register):

    try:
        high = bus.read_byte_data(adr_device, adr_register)
        low = bus.read_byte_data(adr_device, adr_register + 1)
        val = (high << 8) + low
        return val

    except IOError as err:
        print(err)

# Checks if the value is negative and performs 2's Complement if binary value is negative
def read_word_2c(bus, adr_device, adr_register):

    val = read_word(bus, adr_device, adr_register)
    if (val >= 0x8000):
        return -((65535 - val) + 1)

    else:
        return val

# Write data to sensor's registers
def write_byte(bus, adr_device, adr_register, value):

    try:
        bus.write_byte_data(adr_device, adr_register, value)

    except IOError as err:
        print(err)
