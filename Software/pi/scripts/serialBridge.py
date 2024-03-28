import zmq
import serial

# Set up ZeroMQ context and socket
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")  # Use port 5555 for PUB socket

# Set the serial port parameters
serial_port = serial.Serial("/dev/serial0", baudrate=115200, timeout=1)

try:
    while True:
        # Read a line from the serial port
        line = serial_port.readline().decode("utf-8").strip()
        # Encode the topic and data as a multipart message
        topic = "stm"
        message_parts = (topic.encode(), line.encode())

        # Send the multipart message over ZeroMQ
        socket.send_multipart(message_parts)

except KeyboardInterrupt:
    # Close the serial port when the script is interrupted
    serial_port.close()
    print("Serial port closed.")