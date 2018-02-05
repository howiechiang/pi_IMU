#!/usr/bin/python


########################################################################################################################
# SPECIFICATION OF MPU6050 IMU SENSOR
########################################################################################################################

# IMU Register Addresses
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

# DeadZones
accel_deadzone = 8
gyro_deadzone = 2