import cv2
import numpy as np
import argparse
import subprocess

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
        default="udp://0.0.0.0:5000",
        help="UDP stream address (default: udp://0.0.0.0:5005). Only used if --source is udp."
    )
    return parser.parse_args()

def ffmpeg_stream_reader(udp_address):
    """
    Use FFmpeg to read UDP stream, convert it to grayscale, and pipe it to OpenCV.
    Includes low-latency options with frame dropping enabled.
    """
    ffmpeg_command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel", "error",           # Suppress logs for clarity
        "-fflags", "nobuffer",          # Disable buffering for low latency
        "-flags", "low_delay",          # Enable low-latency decoding
        "-i", udp_address,              # Input from UDP stream
        "-vf", "format=gray",           # Convert to grayscale
        "-f", "rawvideo",               # Output raw video data
        "-pix_fmt", "gray",             # Pixel format: grayscale
        "-an",                          # No audio
        "pipe:1"                        # Output to stdout
    ]

    # Start FFmpeg process
    process = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=10**8)
    return process


def main():
    # Parse command-line arguments
    args = parse_args()

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

    if args.source == "udp":
        # Use FFmpeg to handle the UDP stream
        process = ffmpeg_stream_reader(args.udp_address)

        # Define the video frame dimensions (change as per your stream settings)
        width, height = 640, 480  # Adjust based on your stream resolution
        frame_size = width * height  # Grayscale: 1 byte per pixel

        def get_frame():
            """Read a frame from the FFmpeg process."""
            raw_frame = process.stdout.read(frame_size)
            if len(raw_frame) != frame_size:
                return None
            frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((height, width))
            return frame

    else:  # Default to camera input
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            print("Error: Couldn't open the camera.")
            exit()

        def get_frame():
            """Read a frame from the camera."""
            ret, frame = cap.read()
            if not ret:
                print("Error: Couldn't read frame from the camera.")
                return None
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale

    while True:
        frame = get_frame()
        if frame is None:
            break

        # Draw the ROI rectangle on the original frame
        if roi_selected or drawing:
            x1, y1, x2, y2 = roi
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)

        # Display the original frame with the drawn ROI
        cv2.imshow("Original Video", frame)

        if roi_selected:
            # Extract the ROI
            x1, y1, x2, y2 = roi
            roi_frame = frame[min(y1, y2):max(y1, y2), min(x1, x2):max(x1, x2)]
            if roi_frame.size > 0:
                # Apply threshold to the ROI
                _, thresholded_roi = cv2.threshold(roi_frame, 127, 255, cv2.THRESH_BINARY)

                # Display the thresholded ROI
                cv2.imshow("Thresholded ROI", thresholded_roi)

        # Check for the 'q' key to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    if args.source == "udp":
        process.terminate()
    else:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
