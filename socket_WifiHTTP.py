#!/usr/bin/python

import web
from IMU_I2C import IMU


########################################################################################################################
# Initiate Sensors & Webpage Gets
########################################################################################################################
urls = ('/', 'index')
imu1 = IMU(0x68)


class index:

    def GET(self):

        imu1.readall_KF()
        return str(imu1.xRotation) + " " + str(imu1.yRotation) + " " + str(imu1.zRotation)

if __name__ == "__main__":

    imu1.calibrateSensor()
    app = web.application(urls, globals())
    app.run()
