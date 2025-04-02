import cv2
import numpy as np

# Load video
video_path = "IMG_9851.mov"  # Update with actual path
cap = cv2.VideoCapture(video_path)

# Get video properties
fps = int(cap.get(cv2.CAP_PROP_FPS))
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Create a blank canvas for overlaying frames
timelapse = np.zeros((frame_height, frame_width, 3), dtype=np.float32)

# Frame counter
frame_count = 0

# Accumulator for blending
overlay_accumulator = np.zeros((frame_height, frame_width, 3), dtype=np.float32)
blend_factor = 0.3  # Transparency factor per frame

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Select one frame per 3 seconds
    if frame_count % (10 * fps) == 0:
        # Convert frame to float
        frame_float = frame.astype(np.float32)
        
        # Compute luminance normalization factor
        luminance = np.sum(frame_float)
        if luminance > 0:
            frame_normalized = frame_float / luminance
        else:
            frame_normalized = frame_float
        
        # Add normalized frame to accumulator
        overlay_accumulator = overlay_accumulator * (1 - blend_factor) + frame_normalized * blend_factor * luminance
        
        # Normalize accumulated image to maintain brightness consistency
        timelapse = np.clip(overlay_accumulator, 0, 255).astype(np.uint8)
    
    frame_count += 1
    
    # Display timelapse
    cv2.imshow("Motion Timelapse", timelapse)
    
    if cv2.waitKey(30) & 0xFF == 27:  # Press 'ESC' to exit
        break

cap.release()
cv2.destroyAllWindows()

# Save final timelapse frame
cv2.imwrite("motion_timelapse.png", timelapse)
