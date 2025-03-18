import serial,time
import numpy as np

# TFLuna Lidar
sers = []

sers.append(serial.Serial("/dev/ttyAMA1", 115200,timeout=0)) # mini UART serial device
sers.append(serial.Serial("/dev/ttyS0", 115200,timeout=0))

# read ToF data from TF-Luna

def read_tfluna_data(s):
    while True:
        counter = s.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            bytes_serial = s.read(9) # read 9 bytes
            s.reset_input_buffer() # reset buffer

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # check first two bytes
                distance = bytes_serial[2] + bytes_serial[3]*256 # distance in next two bytes
                strength = bytes_serial[4] + bytes_serial[5]*256 # signal strength in next two bytes
                temperature = bytes_serial[6] + bytes_serial[7]*256 # temp in next two bytes
                temperature = (temperature/8.0) - 256.0 # temp scaling and offset
                return distance/100.0,strength,temperature

for s in sers:

    if s.isOpen() == False:
        s.open() # open serial port if not open
    distance,strength,temperature = read_tfluna_data(s) # read values
    print(s)
    print('Distance: {0:2.2f} m, Strength: {1:2.0f} / 65535 (16-bit), Chip Temperature: {2:2.1f} C'.\
                  format(distance,strength,temperature)) # print sample data
    s.close() # close serial port

