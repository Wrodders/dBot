#-----------------------------------------------------------
# @brief Tracks Robot from top-down view using AprilTag detection.
# @details Uses Continuity Camera or Video file as input.

import cv2
import numpy as np
import argparse
import matplotlib.pyplot as plt
import time
import sys
import os
import subprocess
import multiprocessing as mp

# Constants
RESOLUTION = '1920x1080'
FPS = 30
DURATION = 60 
OUTPUT_FILE = 'tracked.mp4'

marker_center = []

def plot_track():
    data = np.loadtxt('track.csv', delimiter=',', dtype=int)
    plt.plot(data[:, 0], data[:, 1])
    plt.title("Track")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

def find_robot(gray, frame, apriltag_dict_16h5, apriltag_dict_36h10, parameters):
    corners_16h5, ids_16h5, _ = cv2.aruco.detectMarkers(gray, apriltag_dict_16h5, parameters=parameters)
    corners_36h10, ids_36h10, _ = cv2.aruco.detectMarkers(gray, apriltag_dict_36h10, parameters=parameters)
    corners = []
    ids = []
    if ids_16h5 is not None:
        corners.extend(corners_16h5)
        ids.extend(ids_16h5.flatten())

    if ids_36h10 is not None:
        corners.extend(corners_36h10)
        ids.extend(ids_36h10.flatten())

    ids = np.array(ids) if ids else None
    if ids is not None:
        valid_corners = []
        valid_centers = []
        for i in range(len(ids)):
            if ids[i] == 0:
                tag_corners = np.int32(corners[i][0])
                center = np.mean(tag_corners, axis=0).astype(int)
                valid_corners.append(tag_corners)
                valid_centers.append(center)
                pt0, pt1 = tag_corners[0], tag_corners[1]
                dx = pt1[0] - pt0[0]
                dy = pt1[1] - pt0[1]
                angle = np.arctan2(dy, dx)
                arrow_length = 50
                end_point = (
                    int(center[0] + arrow_length * np.cos(angle)),
                    int(center[1] + arrow_length * np.sin(angle))
                )

                cv2.polylines(frame, [tag_corners], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.arrowedLine(frame, tuple(center), end_point, (0, 0, 255), 3)
                cv2.putText(frame, f"ID: {ids[i]}", (center[0]-20, center[1]-25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        avg_center = np.mean(valid_centers, axis=0).astype(int) if valid_centers else None
        if avg_center is not None:
            center1 = avg_center
            marker_center.append(center1)
            rect1 = (center1[0] - 100, center1[1] - 100, center1[0] + 100, center1[1] + 100)
            cv2.rectangle(frame, (rect1[0], rect1[1]), (rect1[2], rect1[3]), (255, 0, 0), 2)
            return rect1
    return None

def filterTrack(gray):
    roi = gray[0:1080, 0:1750] 
    roi = cv2.bilateralFilter(roi, 19, 75, 75)
    thresh = cv2.adaptiveThreshold(roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    thresh = cv2.medianBlur(thresh, 5)

    kernel = np.ones((7, 7), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    return thresh

def findTrack(frame, rect, last_track_contours) -> bool:
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
        mask_current = np.zeros(thresh.shape, dtype=np.uint8)
        mask_last = np.zeros(thresh.shape, dtype=np.uint8)
        cv2.drawContours(mask_current, [largest_contour], -1, 255, -1)
        if last_track_contours is not None:
            cv2.drawContours(mask_last, [last_track_contours], -1, 255, -1)
            mask_combined = cv2.bitwise_or(mask_current, mask_last)
            contours_combined, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            largest_contour = max(contours_combined, key=cv2.contourArea)
            if(len(largest_contour) < 0.75*len(last_track_contours)):
                track_found = True
                with open('track.csv', 'w') as f:
                    for point in largest_contour:
                        f.write(f"{point[0][0]},{point[0][1]}\n")
                plot_progress = mp.Process(target=plot_track)
                plot_progress.start()
        last_track_contours = largest_contour
    return last_track_contours, track_found

def main(video_source):
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

    apriltag_dict_16h5 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16H5)
    apriltag_dict_36h10 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H10)
    parameters = cv2.aruco.DetectorParameters()

    last_track_contours = None
    track_found = False

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        robotFrame = frame.copy()
        rect = find_robot(robotFrame, robotFrame, apriltag_dict_16h5, apriltag_dict_36h10, parameters)
        if not track_found:
            last_track_contours, track_found = findTrack(frame, rect, last_track_contours)
        if last_track_contours is not None:
            img = np.zeros(frame.shape, dtype=frame.dtype)
            cv2.drawContours(img, [last_track_contours], -1, (0, 255, 0), 2)
            robotFrame = cv2.addWeighted(img, 0.5, robotFrame, 0.5, 0)

        # Write processed frame to FFmpeg
        try:
            ffmpeg_process.stdin.write(robotFrame.tobytes())
        except BrokenPipeError:
            print("FFmpeg pipe closed.")
            break

        cv2.imshow("Track Detection", robotFrame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    ffmpeg_process.stdin.close()
    ffmpeg_process.wait()
    cv2.destroyAllWindows()

    with open('markers_data.csv', 'w') as f:
        for marker in marker_center:
            f.write(f"{marker[0]},{marker[1]}\n")

    print("Markers data saved to markers_data.csv")
    print("Track detection completed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Track detection with contour averaging.")
    parser.add_argument("--video", type=str, help="Path to input video file.")
    args = parser.parse_args()

    video_source = args.video if args.video else 0
    main(video_source)
