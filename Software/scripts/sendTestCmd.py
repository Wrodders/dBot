import zmq
import csv
import time
import argparse
import numpy as np
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger()

def read_signal_from_csv(csv_file):
    """
    Reads the time and signal data from the specified CSV file.
    Assumes the CSV file has two columns: Time and Signal.
    """
    time_data = []
    signal_data = []

    try:
        with open(csv_file, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row
            for row in reader:
                time_data.append(float(row[0]))  # Time in seconds
                signal_data.append(float(row[1]))  # Signal value
        return np.array(time_data), np.array(signal_data)
    except Exception as e:
        logger.error(f"Error reading CSV file: {e}")
        raise

def setup_publisher(address="tcp://*:5555"):
    """
    Sets up the ZeroMQ PUB socket for sending commands over TCP.
    """
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(address)
    logger.info(f"Publisher bound to {address}")
    return socket

def send_signal_data(socket, time_data, signal_data, sample_period):
    """
    Sends signal data over ZeroMQ using the SERIAL topic.
    """

    initMsg = "<bb1\n"
    socket.send_multipart(["SERIAL".encode(), initMsg.encode()]) 
    start_time = time.time()
    for t, signal in zip(time_data, signal_data):
        # Wait until the correct time to send the next sample
        elapsed_time = time.time() - start_time
        time_to_wait = (t - elapsed_time) if t > elapsed_time else 0
        time.sleep(time_to_wait)
        
        # Send the data with the SERIAL topic
        message = f"{signal}"
        socket.send_string(f"SERIAL {message}")
        logger.info(f"Sent: {message} at {t:.3f}s")

        # Wait for the next sample
        time.sleep(sample_period)

def main():
    parser = argparse.ArgumentParser(description="Send signal data over ZeroMQ to the SERIAL topic.")
    parser.add_argument('csv_file', type=str, help="Path to the CSV file containing time and signal data")
    parser.add_argument('--sample_period', type=float, default=0.1, help="Sample period in seconds (e.g., 0.1 for 100ms)")

    args = parser.parse_args()

    # Read the signal data from the CSV file
    time_data, signal_data = read_signal_from_csv(args.csv_file)

    # Set up the ZeroMQ publisher
    publisher_socket = setup_publisher()

    # Send the signal data at the specified sample period
    send_signal_data(publisher_socket, time_data, signal_data, args.sample_period)

    logger.info("Finished sending all signal data.")

if __name__ == "__main__":
    main()
