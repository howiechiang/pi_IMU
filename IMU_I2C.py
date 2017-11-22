#!/usr/bin/python

from common import *
import smbus
import time
import curses



########################################################################################################################
# SPECIFICATION OF MPU6050 IMU SENSOR
########################################################################################################################
power_mgmt_1 = 0x6b

addr_imu = 0x68
addr_gyro = 0x43
addr_accel = 0x3b

addr_x_gyro = addr_gyro             # gyro_xout address is 67
addr_y_gyro = addr_gyro + 2         # gyro_yout address is 69
addr_z_gyro = addr_gyro + 4         # gyro_zout address is 71

addr_x_accel = addr_accel           # accel_xout address is 59
addr_y_accel = addr_accel + 2       # accel_yout address is 61
addr_z_accel = addr_accel + 4       # accel_zout address is 63

# Sensor Scaling Values
gyro_scale = 131.0
accel_scale = 16384.0
threshLastUpdatedReading = 1       # Signal error if the last reading was taken more than 5 seconds ago..

########################################################################################################################
# KALMAN'S FILTER SETTINGS
########################################################################################################################
K = 0.98
K1 = 1 - K
time_diff = 0.01


class IMU():
    def __init__(self, addr):

        self.address = addr                 # This is the address value read via the i2cdetect command
        self.bus = smbus.SMBus(1)           # or bus = smbus.SMBus(1) for Revision 2 boards

        # Now wake the 6050 up as it starts in sleep mode
        self.bus.write_byte_data(addr, power_mgmt_1, 0)

        self.xRotation = 0
        self.yRotation = 0
        self.zRotation = 0

        self.gyro_xOffset = 0
        self.gyro_yOffset = 0
        self.gyro_zOffset = 0

        self.tLastUpdated = None

    ####################################################################################################################
    # SENSOR COMMUNICATION [I2C]
    ####################################################################################################################

    # # Reads data from sensor's registers
    # def read_byte(self, adr):
    #
    #     try:
    #        return self.bus.read_byte_data(self.address, adr)
    #
    #     except IOError as err:
    #         print(err)
    #
    # def read_word(self, adr):
    #
    #     try:
    #         high = self.bus.read_byte_data(self.address, adr)
    #         low = self.bus.read_byte_data(self.address, adr + 1)
    #         val = (high << 8) + low
    #         return val
    #
    #     except IOError as err:
    #         print(err)
    #
    # # Checks if the value is negative and performs 2's Complement if binary value is negative
    # def read_word_2c(self, adr):
    #
    #     val = self.read_word(adr)
    #     if (val >= 0x8000):
    #         return -((65535 - val) + 1)
    #
    #     else:
    #         return val
    #
    # # Write data to sensor's registers
    # def write_byte(self, adr, value):
    #     try:
    #         self.bus.write_byte_data(self.address, adr, value)
    #
    #     except IOError as err:
    #         print(err)

    # Read all raw data from IMU & Compass
    def get_Data(self):

        raw_gyro_data = None
        raw_accel_data = None
	
	while raw_gyro_data is None and raw_accel_data is None:
	   try:
               raw_gyro_data = self.bus.read_i2c_block_data(self.address, addr_gyro, 6)
               raw_accel_data = self.bus.read_i2c_block_data(self.address, addr_accel, 6)
           except:
	       pass

        gyro_xScaled = twos_compliment((raw_gyro_data[0] << 8) + raw_gyro_data[1]) / gyro_scale
        gyro_yScaled = twos_compliment((raw_gyro_data[2] << 8) + raw_gyro_data[3]) / gyro_scale
        gyro_zScaled = twos_compliment((raw_gyro_data[4] << 8) + raw_gyro_data[5]) / gyro_scale

        accel_xScaled = twos_compliment((raw_accel_data[0] << 8) + raw_accel_data[1]) / accel_scale
        accel_yScaled = twos_compliment((raw_accel_data[2] << 8) + raw_accel_data[3]) / accel_scale
        accel_zScaled = twos_compliment((raw_accel_data[4] << 8) + raw_accel_data[5]) / accel_scale

        return gyro_xScaled, gyro_yScaled, gyro_zScaled, accel_xScaled, accel_yScaled, accel_zScaled

    # Read all raw data and perform Kalman's filter
    # Accelerometer has noise, while the gyro has steady state drift
    def readall_KF(self):

        # This reduces the sampling rate of the data request
        time.sleep(time_diff - 0.005)

        # Update Time
        tNow = time.time()
	if self.tLastUpdated is None:
	    self.tLastUpdated = tNow
	#else:
	#     dt = tNow - self.tLastUpdated
	dt = tNow - self.tLastUpdated
        #if dt > threshLastUpdatedReading: raise ValueError('The last updated reading was over a second ago....)')
        self.tLastUpdated = tNow

        gyro_xScaled, gyro_yScaled, gyro_zScaled, accel_xScaled, accel_yScaled, accel_zScaled \
            = self.get_Data()

        # Subtract offset from raw gyro data. The offset value is determined during calibration
        gyro_xScaled -= self.gyro_xOffset
        gyro_yScaled -= self.gyro_yOffset
        gyro_zScaled -= self.gyro_zOffset

        # d_gyro = gyro * d_t
        gyro_xDelta = (gyro_xScaled * dt)
        gyro_yDelta = (gyro_yScaled * dt)
        gyro_zDelta = (gyro_zScaled * dt)

        # Calculate rotation data from raw accel data
        accel_x = get_x_rotation(accel_xScaled, accel_yScaled, accel_zScaled)
        accel_y = get_y_rotation(accel_xScaled, accel_yScaled, accel_zScaled)
        accel_z = get_z_rotation(accel_xScaled, accel_yScaled, accel_zScaled)

        # Combine both gyro & accel data using Kalman's Filter
        self.xRotation = K * (self.xRotation + gyro_xDelta) + (K1 * accel_x)
        self.yRotation = K * (self.yRotation + gyro_yDelta) + (K1 * accel_y)
        self.zRotation = K * (self.zRotation + gyro_zDelta) + (K1 * accel_z)

        # # gyro_Sum =  gyro_dt
        # # This is something we can track, but it has steady state drift and therefore not reliable data
        # gyro_total_x += gyro_xDelta
        # gyro_total_y += gyro_yDelta
        # gyro_total_z += gyro_zDelta

        return self.xRotation, self.yRotation, self.zRotation

    def calibrate_imu(self):

        # Read raw gyro & accel data from sensors
        self.gyro_xOffset, self.gyro_yOffset, self.gyro_zOffset, accel_xScaled, accel_yScaled, accel_zScaled \
            = self.get_Data()

        # Convert accel data to rotation data
        self.xRotation = get_x_rotation(accel_xScaled, accel_yScaled, accel_zScaled)
        self.yRotation = get_y_rotation(accel_xScaled, accel_yScaled, accel_zScaled)
        self.zRotation = get_z_rotation(accel_xScaled, accel_yScaled, accel_zScaled)


    # This utilizes curses library which clears refreshes the contents of the carriage on the command line
    def display_Data(self):

        stdscr = curses.initscr()

        while True:
            try:

                stdscr.clear()

                #### GYRO DATA ####
                gyro_xout = read_word_2c(self.bus, self.address, addr_x_gyro)
                gyro_yout = read_word_2c(self.bus, self.address, addr_y_gyro)
                gyro_zout = read_word_2c(self.bus, self.address, addr_z_gyro)

                stdscr.addstr("Raw gyro data\n------------------\n")
                stdscr.addstr("gyro_xout: " + str(gyro_xout) + " scaled: " + str(gyro_xout / gyro_scale) + "\n")
                stdscr.addstr("gyro_yout: " + str(gyro_yout) + " scaled: " + str(gyro_yout / gyro_scale) + "\n")
                stdscr.addstr("gyro_zout: " + str(gyro_zout) + " scaled: " + str(gyro_zout / gyro_scale) + "\n")
                stdscr.addstr("\n")

                #### ACCELEROMETER DATA ####
                accel_xout = read_word_2c(self.bus, self.address, addr_x_accel)
                accel_yout = read_word_2c(self.bus, self.address, addr_y_accel)
                accel_zout = read_word_2c(self.bus, self.address, addr_z_accel)

                stdscr.addstr("Raw accelerometer data\n------------------\n")
                stdscr.addstr("accel_xout: " + str(accel_xout) + " scaled: " + str(accel_xout / accel_scale) + "\n")
                stdscr.addstr("accel_yout: " + str(accel_yout) + " scaled: " + str(accel_yout / accel_scale) + "\n")
                stdscr.addstr("accel_zout: " + str(accel_zout) + " scaled: " + str(accel_zout / accel_scale) + "\n")
                stdscr.addstr("\n")

                #### CALCULATED ROTATION DATA ####
                stdscr.addstr("Rotation\n------------------\n")
                stdscr.addstr("x rotation: " + str(get_x_rotation(accel_xout / accel_scale, accel_yout / accel_scale,
                                                                  accel_zout / accel_scale)) + "\n")
                stdscr.addstr("y rotation: " + str(get_y_rotation(accel_xout / accel_scale, accel_yout / accel_scale,
                                                                  accel_zout / accel_scale)) + "\n")
                stdscr.addstr("\n")

                stdscr.refresh()
                time.sleep(1)

            except:

                stdscr.keypad(0)
                curses.echo()
                curses.nocbreak()
                curses.endwin()


########################################################################################################################
# MAIN / DUMP DATA
########################################################################################################################
if __name__ == "__main__":

    imu1 = IMU(addr_imu)
    imu1.display_Data()

    while True:
        imu1.update_KalmanFilter()

