"""
Example script for controlling the KCube101 power meter

Author: Sean Keenan
GitHub: SMK-UK
Date: 13/02/2024

"""

# import the relevant class
from KCube import KinesisMotor

# initialise the device (serial number needs to be string - found on the KDC101)
KCube = KinesisMotor(devicename='PRM1-MZ8', serial_no="27267199")
# move the device to position 200 degs
KCube.move_to(200)
# change the speed (mm/s) and acceleration (mm/s^2) of the device
KCube.set_velocity(max_vel=5)
KCube.set_acceleration(15)
# move to new position
KCube.move_to(150)
# check the new velocity params
KCube.get_vel_params()
# home the device
KCube.go_to_home()
# check position
KCube.get_pos()
# disconnect
KCube.disconnect()

