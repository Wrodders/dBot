import cv2
import numpy as np
import argparse
import matplotlib.pyplot as plt
import time
import sys
import os
import subprocess
import multiprocessing as mp
import math

# Constants
RESOLUTION = '1920x1080'
FPS = 30
DURATION = 60 
OUTPUT_FILE = 'tracked.mp4'

# Global data lists for markers and lap times
# marker_data records tuples: (timestamp, x, y)
marker_data = []  

# Lap tracking variables:
lap_zone_active = False    # Are we already in the "start" zone?
lap_start_timestamp = None # Timestamp when current lap started
lap_count = 0

# Distance threshold in pixels to decide if the robot is crossing the fixed start tag
CROSSING_THRESHOLD = 400

# Fixed start tag center (will be updated if detected)
fixed_tag_center = None

def plot_track():
    """Plot the track.csv data for verification."""
    try:
        data = np.loadtxt('track.csv', delimiter=',')
    except Exception as e:
        print("Unable to load track.csv for plotting:", e)
        return

    plt.figure()
    plt.plot(data[:, 0], data[:, 1], 'k.-')
    plt.title("Track from Contours")
    plt.xlabel("X coordinate")
    plt.ylabel("Y coordinate")
    plt.show()

def find_robot(gray, frame, apriltag_dict_16h5, apriltag_dict_36h10, parameters, timestamp):
    """
    Detects robot tags and returns a rectangle around the averaged center.
    Also logs the (timestamp, x, y) of the marker.
    """
    # Detect robot tags using two dictionaries (top and side tags)
   # corners_16h5, ids_16h5, _ = cv2.aruco.detectMarkers(gray, apriltag_dict_16h5, parameters=parameters)
    corners_36h10, ids_36h10, _ = cv2.aruco.detectMarkers(gray, apriltag_dict_36h10, parameters=parameters)
    
    corners = []
    ids = []
    #if ids_16h5 is not None:
   #     corners.extend(corners_16h5)
    #    ids.extend(ids_16h5.flatten())
    if ids_36h10 is not None:
        corners.extend(corners_36h10)
        ids.extend(ids_36h10.flatten())
    
    ids = np.array(ids) if ids else None
    if ids is not None:
        valid_centers = []
        # Loop over detections. We assume any valid detection from the robot is used
        for i in range(len(ids)):
            # Here you can add extra conditions if needed (e.g. using specific tag sizes)
            tag_corners = np.int32(corners[i][0])
            center = np.mean(tag_corners, axis=0).astype(int)
            valid_centers.append(center)
            
            # Draw detection on the frame
            pt0, pt1 = tag_corners[0], tag_corners[1]
            dx = pt1[0] - pt0[0]
            dy = pt1[1] - pt0[1]
            angle = np.arctan2(dy, dx)
            arrow_length = 50
            end_point = (int(center[0] + arrow_length * np.cos(angle)),
                         int(center[1] + arrow_length * np.sin(angle)))
            
            cv2.polylines(frame, [tag_corners], isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.arrowedLine(frame, tuple(center), end_point, (0, 0, 255), 3)
            cv2.putText(frame, f"ID: {ids[i]}", (center[0]-20, center[1]-25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Compute average center among all valid detections
        avg_center = np.median(valid_centers, axis=0).astype(int)
        if avg_center is not None:
            # Save the detection (timestamp, x, y) for post analysis
            marker_data.append((timestamp, avg_center[0], avg_center[1]))
            rect_center = avg_center
            # Draw a rectangle (for visualization) around the marker area
            rect1 = (rect_center[0] - 100, rect_center[1] - 100,
                     rect_center[0] + 100, rect_center[1] + 100)
            cv2.rectangle(frame, (rect1[0], rect1[1]), (rect1[2], rect1[3]), (255, 0, 0), 2)
            return rect1, avg_center
    return None, None

def detect_fixed_tag(gray, frame, fixed_tag_dict, parameters):
    """
    Detect the fixed start tag (assumed to be a 7x7 tag with id 0).
    Returns the center if detected.
    """
    global fixed_tag_center
    corners_fixed, ids_fixed, _ = cv2.aruco.detectMarkers(gray, fixed_tag_dict, parameters=parameters)
    if ids_fixed is not None:
        for i in range(len(ids_fixed)):
            # Check for fixed tag with id 0
            if ids_fixed[i][0] == 0:
                tag_corners = np.int32(corners_fixed[i][0])
                center = np.mean(tag_corners, axis=0).astype(int)
                fixed_tag_center = center  # update the global fixed tag center
                cv2.polylines(frame, [tag_corners], isClosed=True, color=(255, 0, 255), thickness=2)
                cv2.putText(frame, "Start", (center[0]-20, center[1]-25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                return center
    return fixed_tag_center

def filterTrack(gray):
    """Apply filtering, thresholding, and morphological operations to find the track."""
    roi = gray[0:1080, 0:1750] 
    roi = cv2.bilateralFilter(roi, 19, 75, 75)
    thresh = cv2.adaptiveThreshold(roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY_INV, 11, 2)
    thresh = cv2.medianBlur(thresh, 5)
    kernel = np.ones((7, 7), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    return thresh

def findTrack(frame, rect, last_track_contours) -> (np.array, bool):
    """
    Detects the black track contour. If a valid new track is found (determined by a change in contour size), 
    writes the data to track.csv and plots it.
    """
    track_found = False
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    thresh = filterTrack(gray)
    if rect is not None:
        mask = np.ones(thresh.shape, dtype=np.uint8) * 255
        cv2.rectangle(mask, (rect[0], rect[1]), (rect[2], rect[3]), (0, 0, 0), -1)
        thresh = cv2.bitwise_and(thresh, mask)
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        mask_current = np.zeros(thresh.shape, dtype=thresh.dtype)
        mask_last = np.zeros(thresh.shape, dtype=thresh.dtype)
        cv2.drawContours(mask_current, [largest_contour], -1, 255, -1)
        if last_track_contours is not None:
            cv2.drawContours(mask_last, [last_track_contours], -1, 255, -1)
            mask_combined = cv2.bitwise_or(mask_current, mask_last)
            contours_combined, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            largest_contour = max(contours_combined, key=cv2.contourArea)
            if len(largest_contour) < 0.75 * len(last_track_contours):
                track_found = True
                with open('track.csv', 'w') as f:
                    for point in largest_contour:
                        f.write(f"{point[0][0]},{point[0][1]}\n")
                # Plotting in a separate process so as not to block the video stream.
                plot_progress = mp.Process(target=plot_track)
                plot_progress.start()
        last_track_contours = largest_contour
    return last_track_contours, track_found

def main(video_source):
    global lap_zone_active, lap_start_timestamp, lap_count

    cap = cv2.VideoCapture(video_source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    ffmpeg_cmd = [
        'ffmpeg', '-y',
        '-f', 'rawvideo',
        '-vcodec', 'rawvideo',
        '-pix_fmt', 'bgr24',
        '-s', RESOLUTION,
        '-r', str(FPS),
        '-i', '-',
        '-an',
        '-vcodec', 'libx264',
        '-preset', 'fast',
        OUTPUT_FILE
    ]
    ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

    # Create dictionaries for robot tags and fixed tag
    apriltag_dict_16h5 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16H5)
    apriltag_dict_36h10 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H10)
    # For the fixed starting tag, we assume a 7x7 family exists.
    try:
        fixed_tag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_7X7)
    except:
        # Fallback: if not available, you can try creating one manually or use a different dictionary.
        fixed_tag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    parameters = cv2.aruco.DetectorParameters()

    last_track_contours = None
    track_found = False

    # Initialize lap start timestamp once the video begins (or when the first lap crossing occurs)
    lap_start_timestamp = None

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Grab current timestamp (in milliseconds)
        timestamp = cap.get(cv2.CAP_PROP_POS_MSEC)

        robotFrame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. Detect the fixed starting tag for lap detection
        current_fixed_center = detect_fixed_tag(gray, robotFrame, fixed_tag_dict, parameters)

        # 2. Detect robot markers and get averaged marker center
        rect, robot_center = find_robot(gray, robotFrame, apriltag_dict_16h5, apriltag_dict_36h10, parameters, timestamp)

        # 3. Check for lap completion.
        #    If both a fixed tag and robot marker are detected, check distance.
        if current_fixed_center is not None and robot_center is not None:
            distance = math.hypot(robot_center[0] - current_fixed_center[0],
                                  robot_center[1] - current_fixed_center[1])
            # If within the threshold and we are not already in the crossing zone
            if distance < CROSSING_THRESHOLD and not lap_zone_active:
                # If this is the first crossing, initialize lap_start_timestamp.
                if lap_start_timestamp is None:
                    lap_start_timestamp = timestamp
                else:
                    # A lap has been completed.
                    lap_time = timestamp - lap_start_timestamp
                    lap_count += 1
                    print(f"Lap complete ({lap_count}): Lap time = {lap_time:.2f} ms")
                    # Restart lap timer
                    lap_start_timestamp = timestamp
                lap_zone_active = True
            # When robot moves away, clear the flag.
            elif distance >= CROSSING_THRESHOLD:
                lap_zone_active = False

        # 4. Track the black line (track detection) and update contour if track is detected.
        if not track_found:
            last_track_contours, track_found = findTrack(frame, rect, last_track_contours)
        if last_track_contours is not None:
            overlay = np.zeros(frame.shape, dtype=frame.dtype)
            cv2.drawContours(overlay, [last_track_contours], -1, (0, 255, 0), 2)
            robotFrame = cv2.addWeighted(overlay, 0.5, robotFrame, 0.5, 0)

        # Write processed frame to FFmpeg for output video
        try:
            ffmpeg_process.stdin.write(robotFrame.tobytes())
        except BrokenPipeError:
            print("FFmpeg pipe closed.")
            break

        cv2.imshow("Track and Lap Detection", robotFrame)
        if cv2.waitKey(1) == ord('q'):
            break

    # Clean up video capture and windows
    cap.release()
    ffmpeg_process.stdin.close()
    ffmpeg_process.wait()
    cv2.destroyAllWindows()

    # Save robot markers data (timestamp, x, y) to CSV
    with open('markers_data.csv', 'w') as f:
        for entry in marker_data:
            f.write(f"{entry[0]},{entry[1]},{entry[2]}\n")
    print("Markers data saved to markers_data.csv")
    print("Track detection and lap timing completed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot tracking with lap detection and performance evaluation.")
    parser.add_argument("--video", type=str, help="Path to input video file.")
    args = parser.parse_args()

    video_source = args.video if args.video else 0
    main(video_source)
