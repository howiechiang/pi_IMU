#!/usr/bin/python
import smbus
import math
import time

########################################################################################################################
# SPECIFICATIONS
########################################################################################################################
power_mgmt_1 = 0x6b

class COMPASS:
    def __init__(self, addr):

        self.address = addr
        self.bus = smbus.SMBUS(1)           # or bus = smbus.SMBus(1) for Revision 2 boards

        # Now wake the 6050 up as it starts in sleep mode
        self.bus.write_byte_data(addr, power_mgmt_1, 0)



    ####################################################################################################################
    # SENSOR COMMUNICATION [I2C]
    ####################################################################################################################

    # Reads data from sensor's registers
    def read_byte(self, adr):

        try:
           return self.bus.read_byte_data(self.address, adr)

        except IOError as err:
            print(err)

    def read_word(self, adr):

        try:
            high = self.bus.read_byte_data(self.address, adr)
            low = self.bus.read_byte_data(self.address, adr + 1)
            val = (high << 8) + low
            return val

        except IOError as err:
            print(err)

    # Checks if the value is negative and performs 2's Complement if binary value is negative
    def read_word_2c(self, adr):

        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)

        else:
            return val

    # Write data to sensor's registers
    def write_byte(self, adr, value):

        try:
            self.bus.write_byte_data(self.address, adr, value)

        except IOError as err:
            print(err)

    # Write desired settings to Compass Registers
    def compassConfig(self):

        self.bus.write_byte(0, 0b01110000)  # Set to 8 samples @ 15Hz
        self.bus.write_byte(1, 0b00100000)  # 1.3 gain LSb / Gauss 1090 (default)
        self.bus.write_byte(2, 0b00000000)  # Continuous sampling

    def get_CompassRotation(self, x, y):

        radians = math.atan2(y, x)
        if radians < 0:
            radians += 2 * math.pi
        return math.degrees(radians)









# Calibrate Compass Sensor
def calibrate_comp():
    print "COMPASS CALIBRATION START"

    minx = 0
    maxx = 0
    miny = 0
    maxy = 0

    # Find the min & max values for the x & y sensors
    # (The compass must be rotated 360 degrees in the duration of calibration sequence)
    for i in range(0, 500):
        x_out, y_out, z_out = readall_comp()

        if x_out < minx:
            minx = x_out

        if y_out < miny:
            miny = y_out

        if x_out > maxx:
            maxx = x_out

        if y_out > maxy:
            maxy = y_out

        print minx, miny, maxx, maxy
        time.sleep(0.1)

    # X & Y offset are calculated by the average of min & max
    x_offset = (maxx + minx) / 2
    y_offset = (maxy + miny) / 2

    print "COMPASS CLAIBRATION COMPLETED"
    print "X Offset: %.2f" % x_offset
    print "Y Offset: %.2f" % y_offset

    return x_offset, y_offset

# Read Compass data and convert z-axis rotation (with offsets included)
def comp_rotation(x_offset, y_offset):
    comp_scaled_x, comp_scaled_y, comp_scaled_z = readall_comp()

    x_out = (comp_scaled_x - x_offset) * comp_scale
    y_out = (comp_scaled_y - y_offset) * comp_scale

    return get_comp_rotation(x_out, y_out)


def readall_comp():

    raw_comp_data = bus.read_i2c_block_data(i2c_address, comp_address, 9)

    comp_scaled_x = read_word_2c(comp_address, 3) * comp_scale
    comp_scaled_y = read_word_2c(comp_address, 5) * comp_scale
    comp_scaled_z = read_word_2c(comp_address, 7) * comp_scale

    return comp_scaled_x, comp_scaled_y, comp_scaled_z