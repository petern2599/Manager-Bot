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
    alt = 0
    while True:
        time.sleep(1)
        if alt == 0:
            serial_conn.write("Hello from RPi\n".encode('utf-8'))
            alt = 1
        else:
            serial_conn.write("Hello from Raspberry\n".encode('utf-8'))
            alt = 0
except KeyboardInterrupt:
    # Close serial connection
    serial_conn.close()
    print("Serial Connection Closed")