import cv2
import numpy as np

# UDP stream address and port
udp_address = '192.168.1.122'
udp_port = 5000

# Create a UDP capture object
cap = cv2.VideoCapture(f'udp://{udp_address}:{udp_port}')

# Check if the video stream is successfully opened
if not cap.isOpened():
    print("Error: Couldn't open the video stream.")
    exit()

# Create a window to display the video
cv2.namedWindow("Original Video", cv2.WINDOW_NORMAL)
cv2.namedWindow("Canny Edges", cv2.WINDOW_NORMAL)

while True:
    # Read a frame from the video stream
    ret, frame = cap.read()

    # If the frame is not successfully read, break the loop
    if not ret:
        print("Error: Couldn't read frame.")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Canny edges filter
    edges = cv2.Canny(gray, 50, 150)  # Adjust the threshold values as needed

    # Display the original and edges frames
    cv2.imshow("Original Video", frame)
    cv2.imshow("Canny Edges", edges)

    # Check for the 'q' key to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close the windows
cap.release()
cv2.destroyAllWindows()
