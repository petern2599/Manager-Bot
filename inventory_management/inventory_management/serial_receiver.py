#!/usr/bin/env python3
import serial
import time

# Initiate serial connection
serial_conn = serial.Serial('/dev/ttyACM0',115200,timeout=1.0)

# Wait 3 seconds for Arduino to restart
time.sleep(3)

# Reset input buffer
serial_conn.reset_input_buffer()

print("Serial Connection Established")

try:
    while True:
        time.sleep(0.01)
        #If receiving any bytes from Arduino
        if serial_conn.in_waiting > 0:
            line = serial_conn.readline().decode('utf-8')
            print(line)
except KeyboardInterrupt:
    # Close serial connection
    serial_conn.close()
    print("Serial Connection Closed")