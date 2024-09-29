import serial
import time

# Configure the serial port
port = '/dev/serial0'  # Replace with your serial port (e.g., '/dev/ttyUSB0' for Linux)
baudrate = 115200  # Set the baud rate (must match the device's setting)

# Create a serial connection
ser = serial.Serial(port, baudrate)

# Give some time for the connection to establish
time.sleep(2)

try:
    while True:
        if ser.in_waiting > 0:  # Check if there is data to read
            line = ser.readline()  # Read a line of data
            try:
                print(line.decode('utf-8').strip())  # Print the decoded line
            except Exception as e:
                print(e)
except KeyboardInterrupt:
    print("Exiting...")

finally:
    ser.close()  # Close the serial port