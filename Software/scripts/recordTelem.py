import argparse
import zmq
import os
import csv
import time
from datetime import datetime

def get_timestamp_hms():
    """Get the current timestamp in HH:MM:SS format."""
    return datetime.now().strftime("%H:%M:%S")

def write_csv_headers(file_path, headers, timestamp):
    """Write headers to the CSV file if it's empty or new."""
    if os.path.exists(file_path):
        os.remove(file_path)  # Remove the file if it already exists
    with open(file_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([f"Timestamp: {timestamp}"])
        writer.writerow(headers)

def write_data_to_csv(file_path, data):
    """Write data to the CSV file."""
    with open(file_path, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(data)

def rotate_file(file_path, max_size):
    """Check file size and rotate if it exceeds max_size."""
    if os.path.exists(file_path) and os.path.getsize(file_path) > max_size:
        base, ext = os.path.splitext(file_path)
        rotated_file = f"{base}_{int(time.time())}{ext}"
        os.rename(file_path, rotated_file)

def load_topics_from_case_file(case_file):
    """Load topics from the case file."""
    topics = []
    with open(case_file, 'r') as file:
        for line in file:
            if line.strip():  # Skip empty lines
                parts = line.split(',')
                if len(parts) > 2 and parts[2]:  # Ensure there's a format column
                    topics.append((parts[0], parts[2].strip().split(':')))
    return topics

def main():
    parser = argparse.ArgumentParser(description="Logger via ZMQ.")
    parser.add_argument("--endpoint", type=str, required=True, help="ZMQ tcp endpoint (e.g., tcp://127.0.0.1:5555).")
    parser.add_argument("--output", type=str, required=True, help="Output CSV file path.")
    parser.add_argument("--topics", type=str, required=True, help="Path to the csv of Topics.")
    parser.add_argument("--max-size", type=int, default=10 * 1024 * 1024, help="Maximum file size in bytes before rotating (default: 10MB).")
    args = parser.parse_args()

    # Load topics from the case file
    topics = load_topics_from_case_file(args.topics)

    # Initialize ZMQ context and subscriber socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(args.endpoint)

    topicNames =  [f"{topic}/{subtopic}" for topic, subtopics in topics for subtopic in subtopics]

    # Subscribe to all subtopics
    for topicName in topicNames:
        socket.setsockopt_string(zmq.SUBSCRIBE, topicName)


    print(f"Subscribed to topics from case file on {args.endpoint}")

    # CSV headers
    headers = ["Timestamp_HMS"] + topicNames

    # Initialize the current row with zeros
    current_row = {topic: "INV" for topic in topicNames}
    rowvalid = False


    # Write CSV headers with the current timestamp
    write_csv_headers(args.output, headers, datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

    try:
        while True:
            # Receive ZMQ message
            topic, message = socket.recv_multipart()
            topic_decoded = topic.decode()
            message_decoded = message.decode()
        
            # Update the current row with the received message

            if(current_row[topic_decoded] == 0):
                current_row[topic_decoded] = message_decoded

            # Check if all topics have been received
            rowvalid = all([current_row[topic] != "INV" for topic in topicNames])
            
            # If all topics have been received, write the current row to the CSV file
            if rowvalid:
                print(f"Writing to CSV: {current_row}")
                write_data_to_csv(args.output, [get_timestamp_hms()] + [current_row[topic] for topic in topicNames])
                current_row = {topic: "INV" for topic in topicNames}
                rowvalid = False

            # Rotate the CSV file if it exceeds the maximum size

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    main()
