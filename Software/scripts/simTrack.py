import cv2
import numpy as np
import random
import time
import argparse

def generate_frame(angle):
    """Generate a frame with a white line rotated at a given angle."""
    # Create a 120p black image
    frame = np.zeros((20, 60), dtype=np.uint8)

    # Define the line properties
    line_length = 20
    line_thickness = 5

    # Calculate the center of the image
    center = (frame.shape[1] // 2, frame.shape[0] // 2)

    # Define line's starting position (at the center of the image)
    end_point = (int(center[0] + line_length * np.cos(np.radians(angle))),
                 int(center[1] + line_length * np.sin(np.radians(angle))))

    # Draw the line on the image
    cv2.line(frame, center, end_point, 255, line_thickness)

    return frame

def simulate_track(output_video_path, duration=10, angle_range=(-15, 15)):
    """Simulate a track with random line angle changes and record it to a video file."""
    # Video writer setup with the 'mp4v' codec for .mp4 files
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'mp4v' for MP4 format
    out = cv2.VideoWriter(output_video_path, fourcc, 30.0, (160, 120))  # 30 fps, 120x160 resolution

    # Track simulation parameters
    current_angle = 0  # Start the line at 0 degrees
    start_time = time.time()

    # Simulate the track and capture frames for video
    while time.time() - start_time < duration:
        # Generate the next frame with a random angle within the specified range
        angle_change = random.uniform(angle_range[0], angle_range[1])
        current_angle += angle_change

        # Keep the angle within 360 degrees
        current_angle = current_angle % 360

        # Generate the frame
        frame = generate_frame(current_angle)

        # Write the frame to the video
      
        
        out.write(frame)

        # Optionally display the frame in a window (uncomment to see in real-time)
        cv2.imshow('Track Simulation', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video writer and close any open windows
    out.release()
    print(f"Video saved to {output_video_path}")
    
    # After video creation, play the video
    play_video(output_video_path)

def play_video(video_path):
    """Play the generated video using OpenCV."""
    cap = cv2.VideoCapture(video_path)  # Open the video file
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        return

    # Play the video
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow("Track Simulation", frame)
        
        # Wait for 'q' to quit the video playback
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and close the window
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Simulate a track and save to video")
    parser.add_argument('output', type=str, help="Path to save the output video")
    parser.add_argument('--duration', type=int, default=10, help="Duration of the simulation in seconds")
    parser.add_argument('--angle-range', type=int, nargs=2, default=[-15, 15], help="Range of angle change per frame (default: -15 to 15 degrees)")
    
    args = parser.parse_args()

    # Start the simulation
    simulate_track(args.output, duration=args.duration, angle_range=tuple(args.angle_range))
