import zmq

def setup_zmq_subscriber():
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://192.168.0.39:5555")  # Replace <Raspberry_Pi_IP> with actual IP
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics
    return subscriber

def receive_data(subscriber):
    while True:
        message = subscriber.recv_multipart()
        
        print(message)

if __name__ == "__main__":
    subscriber = setup_zmq_subscriber()
    receive_data(subscriber)