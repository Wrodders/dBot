import serial

# Set the serial port parameters
serial_port = serial.Serial("/dev/serial0", baudrate=115200, timeout=1)

try:
    while True:
        # Read a line from the serial port
        line = serial_port.readline().decode("utf-8").strip()

        # Print the received line
        print(line)

except KeyboardInterrupt:
    # Close the serial port when the script is interrupted
    serial_port.close()
    print("Serial port closed.")
