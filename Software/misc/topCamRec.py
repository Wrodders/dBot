#----------------------------------------------------------
# @brief: Record video from iPhone Continuity Camera using FFmpeg

import cv2
import subprocess
import time
import signal
import sys
import os

CAMERA_INDEX = 1  # Change if needed
RESOLUTION = "1920x1080"
FPS = 30
DURATION = 60  # Seconds

def get_next_filename(base_name="output", ext=".mp4"):
    i = 1
    while os.path.exists(f"{base_name}_{i}{ext}"):
        i += 1
    return f"{base_name}_{i}{ext}"

OUTPUT_FILE = get_next_filename()

ffmpeg_cmd = [
    'ffmpeg', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
    '-s', RESOLUTION, '-r', str(FPS), '-i', '-',
    '-c:v', 'libx264', '-preset', 'fast', '-t', str(DURATION),
    OUTPUT_FILE
]

def cleanup(cap, ffmpeg_proc):
    print("\nStopping recording...")
    if cap.isOpened():
        cap.release()
    if ffmpeg_proc:
        try:
            ffmpeg_proc.stdin.close()
            ffmpeg_proc.wait(timeout=5)
        except (BrokenPipeError, subprocess.TimeoutExpired):
            ffmpeg_proc.terminate()
    cv2.destroyAllWindows()
    print(f"Recording stopped. Video saved as: {OUTPUT_FILE}")

def signal_handler(sig, frame, cap, ffmpeg_proc):
    print("\nUser interrupted, shutting down...")
    cleanup(cap, ffmpeg_proc)
    sys.exit(0)

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Error: Could not open camera at index {CAMERA_INDEX}.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    try:
        ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)
    except FileNotFoundError:
        print("Error: FFmpeg not found. Ensure it is installed and in PATH.")
        sys.exit(1)

    signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, cap, ffmpeg_proc))
    signal.signal(signal.SIGTERM, lambda s, f: signal_handler(s, f, cap, ffmpeg_proc))

    start_time = time.time()

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Error: Lost connection to the camera.")
                break
            cv2.imshow('iPhone Camera', frame)
            try:
                ffmpeg_proc.stdin.write(frame.tobytes())
            except BrokenPipeError:
                print("Error: FFmpeg process has unexpectedly terminated.")
                break
            if time.time() - start_time > DURATION or cv2.waitKey(1) & 0xFF == ord('q'):
                print("User requested stop.")
                break
    finally:
        cleanup(cap, ffmpeg_proc)

if __name__ == "__main__":
    main()
