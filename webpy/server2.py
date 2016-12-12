
#!/usr/bin/python
import web
import smbus
import math
import time


########################################################################################################################
# Web Variables
########################################################################################################################
urls = (
    '/', 'index'
)

########################################################################################################################
# VARIABLES
########################################################################################################################
# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

# Sensor Scaling Values
gyro_scale = 131.0
accel_scale = 16384.0
comp_scale = 0.92

# Address Declaration
bus = smbus.SMBus(1)    # or bus = smbus.SMBus(1) for Revision 2 boards
i2c_address = 0x68          # This is the address value read via the i2cdetect command
gyro_address = 0x43
accel_address = 0x3b
comp_address = 0x1e
# x_data_address = [0 1]
# y_data_address = [2 3]
# z_data_address = [4 5]


########################################################################################################################
# FUNCTIONS
########################################################################################################################

# Read data from sensors, run through kalman filter, and return x,y,z rotation
def kalman_filter():
    # Filter Settings
    K = 0.98
    K1 = 1 - K
    time_diff = 0.01

    now = time.time()

    # Read raw gyro & accel data from sensors
    gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z = readall()

    # Convert accel data to rotation data
    last_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    last_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    last_z = get_z_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)


    gyro_offset_x = gyro_scaled_x
    gyro_offset_y = gyro_scaled_y
    gyro_offset_z = gyro_scaled_z

    gyro_total_x = last_x - gyro_offset_x
    gyro_total_y = last_y - gyro_offset_y
    gyro_total_z = last_z - gyro_offset_z

    # Please have someone explain why this needs to be in a loop and why 0.005 is subtracked frmo time_diff
    for i in range(0, int(3.0 / time_diff)):
        time.sleep(time_diff - 0.005)

        # Read raw gyro & accel data from sensors
        gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z = readall()

        # Subtract offset(first set of read gyro data) from raw gyro data
        gyro_scaled_x -= gyro_offset_x
        gyro_scaled_y -= gyro_offset_y
        gyro_scaled_z -= gyro_offset_z

        # Δgyro = gyro * Δt
        gyro_x_delta = (gyro_scaled_x * time_diff)
        gyro_y_delta = (gyro_scaled_y * time_diff)
        gyro_z_delta = (gyro_scaled_z * time_diff)

        # gyro_sum =  Δgyro
        gyro_total_x += gyro_x_delta
        gyro_total_y += gyro_y_delta
        gyro_total_z += gyro_z_delta

        # Calculate rotation data from raw accel data
        rotation_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
        rotation_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
        rotation_z = get_z_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)

        # Combine both gyro & accel data using Kalman's Filter
        last_x = K * (last_x + gyro_x_delta) + (K1 * rotation_x)
        last_y = K * (last_y + gyro_y_delta) + (K1 * rotation_y)
        last_z = K * (last_z + gyro_z_delta) + (K1 * rotation_z)

        return last_x, last_y, last_z

# Read all raw data from IMU & Compass
def readall():

    raw_gyro_data = bus.read_i2c_block_data(i2c_address, gyro_address, 6)
    raw_accel_data = bus.read_i2c_block_data(i2c_address, accel_address, 6)

    gyro_scaled_x = twos_compliment((raw_gyro_data[0] << 8) + raw_gyro_data[1]) / gyro_scale
    gyro_scaled_y = twos_compliment((raw_gyro_data[2] << 8) + raw_gyro_data[3]) / gyro_scale
    gyro_scaled_z = twos_compliment((raw_gyro_data[4] << 8) + raw_gyro_data[5]) / gyro_scale

    accel_scaled_x = twos_compliment((raw_accel_data[0] << 8) + raw_accel_data[1]) / accel_scale
    accel_scaled_y = twos_compliment((raw_accel_data[2] << 8) + raw_accel_data[3]) / accel_scale
    accel_scaled_z = twos_compliment((raw_accel_data[4] << 8) + raw_accel_data[5]) / accel_scale

    return gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z

# Perform 2's Compliment on hex data. This checks if value is negative in hex representation and converts it.
def twos_compliment(val):
    if val >= 0x8000:
        return-((65535 - val) + 1)
    else:
        return val

# Computational Math Functions
def dist(a,b):
    return math.sqrt((a * a) + (b * b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def get_z_rotation(x,y,z):
    radians = math.atan2(z, dist(x,y))
    return math.degrees(radians)


# Obselete Functions
def read_byte(adr):
    return bus.read_byte_data(i2c_address, adr)

def read_word(adr):
    high = bus.read_byte_data(i2c_address, adr)
    low = bus.read_byte_data(i2c_address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val




class index:
    def GET(self):

        gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z = readall()
        #return str(get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z))+" "+str(get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z))
        last_x, last_y, last_z = kalman_filter()
        return str(last_x) + " " + str(last_y) + " " + str(last_z)
        
        #accel_xout = read_word_2c(0x3b)
        #accel_yout = read_word_2c(0x3d)
        #accel_zout = read_word_2c(0x3f)

        #accel_xout_scaled = accel_xout / 16384.0
        #accel_yout_scaled = accel_yout / 16384.0
        #accel_zout_scaled = accel_zout / 16384.0

        #return str(get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))+" "+str(get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
        

if __name__ == "__main__":

    # Now wake the 6050 up as it starts in sleep mode
    bus.write_byte_data(i2c_address, power_mgmt_1, 0)

    app = web.application(urls, globals())
    app.run()

