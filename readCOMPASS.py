#!/usr/bin/python
import smbus
import time
import math


########################################################################################################################
# VARIABLES
########################################################################################################################

# bus = smbus.SMBus(1) for Revision 2 boards
bus = smbus.SMBus(1)

# This is the address value read via the i2cdetect command for the compass
address = 0x1e

## adr
# x_data_address = 3
# y_data_address = 7
# z_data_address = 5

# Scaled value of sensor reading
scale = 0.92


########################################################################################################################
# FUNCTIONS
########################################################################################################################

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def write_byte(adr, value):
    bus.write_byte_data(address, adr, value)


# Compass Configuration Setup
write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
write_byte(2, 0b00000000) # Continuous sampling


x_out = read_word_2c(3) * scale
y_out = read_word_2c(7) * scale
z_out = read_word_2c(5) * scale


# CALIBRATION
minx = 0
maxx = 0
miny = 0
maxy = 0


print("CALIBRATION START")

for i in range (0,500):
    x_out = read_word_2c(3)
    y_out = read_word_2c(7)
    z_out = read_word_2c(5)

    if x_out < minx:
        minx=x_out

    if y_out < miny:
        miny=y_out

    if x_out > maxx:
        maxx=x_out

    if y_out > maxy:
        maxy=y_out

    time.sleep(0.1)

print("CALIBRATION END")

x_offset = (maxx + minx) / 2
y_offset = (maxy + miny) / 2

x_out = (read_word_2c(3) - x_offset) * scale
y_out = (read_word_2c(7) - y_offset) * scale
z_out = (read_word_2c(5)) * scale

print("x: ", x_out)
print("y: ", y_out)
print("x: ", z_out)


bearing  = math.atan2(y_out, x_out)
if (bearing < 0):
    bearing += 2 * math.pi

print("Bearing: ", math.degrees(bearing))
