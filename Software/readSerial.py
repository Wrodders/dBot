import serial
import time
import zmq
import threading
from collections import namedtuple

Topic = namedtuple('Topic', ['pub_id', 'name'])

pubMap = [
    Topic('a', "CMD_RET"),
    Topic('b', "ERROR"),
    Topic('c', "INFO"),
    Topic('d', "DEBUG"),
    Topic('e', "IMU"),  # ROLL:PITCH:YAW
    Topic('f', "ODOM", )  # LSPEED:RSPEED:TL:TR
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
                msg_frame = ser.readline()
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

def setup_zmq_publisher():
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PUB)
    zmq_socket.bind("tcp://*:5555")  # Publishing on port 5555
    return zmq_socket

if __name__ == "__main__":
    zmq_socket = setup_zmq_publisher()
    start_serial_reader('/dev/serial0', zmq_socket)
