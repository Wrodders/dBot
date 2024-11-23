import cv2
import numpy as np
import argparse

def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Select ROI from video feed and apply threshold. Exit the program by pressing 'q'."
    )
    parser.add_argument(
        "--source",
        type=str,
        choices=["udp", "camera"],
        default="camera",
        help="Video source: 'udp' for UDP stream or 'camera' for the default local camera (default: camera)."
    )
    parser.add_argument(
        "--udp_address",
        type=str,
        default="192.168.1.122",
        help="UDP stream address (default: 192.168.1.122). Only used if --source is udp."
    )
    parser.add_argument(
        "--udp_port",
        type=int,
        default=5000,
        help="UDP stream port (default: 5000). Only used if --source is udp."
    )
    return parser.parse_args()

def main():
    # Parse command-line arguments
    args = parse_args()

    # Determine video source
    if args.source == "udp":
        video_source = f'udp://{args.udp_address}:{args.udp_port}'
    else:  # Default to camera
        video_source = 1

    # Create a video capture object
    cap = cv2.VideoCapture(video_source)

    # Check if the video stream is successfully opened
    if not cap.isOpened():
        print("Error: Couldn't open the video stream or camera.")
        exit()

    # Set the desired resolution
  
    desired_height = 240  # Height in pixels

    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

    # Confirm the actual resolution set by the camera
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera resolution set to: {actual_width}x{actual_height}")

    # Global variables for ROI selection
    roi_selected = False
    roi = (0, 0, 0, 0)
    drawing = False

    def select_roi(event, x, y, flags, param):
        """Mouse callback function to select ROI."""
        nonlocal roi, roi_selected, drawing
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            roi = (x, y, x, y)  # Initialize ROI as (x1, y1, x2, y2)
        elif event == cv2.EVENT_MOUSEMOVE and drawing:
            roi = (roi[0], roi[1], x, y)  # Update ROI while dragging
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False
            roi = (roi[0], roi[1], x, y)  # Finalize ROI
            roi_selected = True

    # Create a window and set the mouse callback for ROI selection
    cv2.namedWindow("Original Video", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Original Video", select_roi)

    while True:
        # Read a frame from the video stream
        ret, frame = cap.read()

        # If the frame is not successfully read, break the loop
        if not ret:
            print("Error: Couldn't read frame.")
            break

        # Draw the ROI rectangle on the original frame
        if roi_selected or drawing:
            x1, y1, x2, y2 = roi
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Display the original frame with the drawn ROI
        cv2.imshow("Original Video", frame)

        if roi_selected:
            # Extract the ROI and convert it to grayscale
            x1, y1, x2, y2 = roi
            roi_frame = frame[min(y1, y2):max(y1, y2), min(x1, x2):max(x1, x2)]
            if roi_frame.size > 0:
                gray_roi = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)

                # Apply threshold to the ROI
                _, thresholded_roi = cv2.threshold(gray_roi, 127, 255, cv2.THRESH_BINARY)

                # Display the thresholded ROI
                cv2.imshow("Thresholded ROI", thresholded_roi)

        # Check for the 'q' key to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object and close the windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
