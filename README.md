# MLX90640-python
Native python driver for Melexis MLX90640 IR Thermal Sensor. Uses smbus2. Designed and tested on Raspberry Pi

Usage:

import MLX90640

sensor = MLX90640() #can optionally include address as argument, default is 0x33

sensor.getCompensatedPixData(i,j) #gets compensated temperature data from chosen pixel from 32x24 pixel array

-----------------------------------------------------------------------------------------------------------------

This is a very rudimentary library at the moment. Hopefully to be updated in the future.
