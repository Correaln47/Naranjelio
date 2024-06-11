from altimu10v5.lsm6ds33 import LSM6DS33
from time import sleep

lsm6ds33 = LSM6DS33()
lsm6ds33.enable()

while True:
    print(lsm6ds33.get_accelerometer_g_forces())
    print(lsm6ds33.get_gyro_angular_velocity())
    sleep(1)