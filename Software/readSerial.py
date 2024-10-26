import serial
import time
import zmq
import threading
import sys
from collections import namedtuple

Topic = namedtuple('Topic', ['pub_id', 'name'])

pubMap = [
    Topic('a', "CMD_RET"),
    Topic('b', "ERROR"),
    Topic('c', "INFO"),
    Topic('d', "DEBUG"),
    Topic('e', "IMU"),  
    Topic('f', "ODOM")
]

def get_topic_by_id(pub_id):
    for topic in pubMap:
        if topic.pub_id == pub_id:
            return topic.name
    return None

def serial_reader(ser, zmq_socket):
    while True:
        if ser.readable():
            try:
                msg_frame = ser.read_until(b'\n')
                msg = msg_frame.decode('utf-8').rstrip("\n")
                msg_id = msg[1]
                msg_data = msg[2:]
                topic = get_topic_by_id(msg_id)
                if topic:
                    zmq_socket.send_multipart([topic.encode(), msg_data.encode()])
            except Exception as e:
                print(f"Serial read error: {e}")

def start_serial_reader(ser_port, zmq_socket):
    ser = serial.Serial(ser_port, 115200)
    threading.Thread(target=serial_reader, args=(ser, zmq_socket)).start()
    return ser

def setup_zmq_publisher():
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PUB)
    zmq_socket.bind("tcp://127.0.0.1:5555")  # Publishing on port 5555
    return zmq_socket

def zmq_subscriber(self, publisher_ip):
    context = zmq.Context()
    zmq_socket = context.socket(zmq.SUB)
    zmq_socket.connect(f"tcp://{publisher_ip}:5555")  # Connect to the publisher's IP
    zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics

    while True:
        try:
            topic, msg_data = zmq_socket.recv_multipart()
            ser.write(msg_data)
        except Exception as e:
            print(f"Subscriber error: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python script.py <serial_port> <publisher_ip>")
        sys.exit(1)

    serial_port = sys.argv[1]
    publisher_ip = sys.argv[2]

    zmq_socket = setup_zmq_publisher()
    ser = start_serial_reader(serial_port, zmq_socket)

    # Start the subscriber in a new thread
    threading.Thread(target=zmq_subscriber, args=(ser, publisher_ip)).start()
