import cv2
import numpy as np

# Load video
video_path = "IMG_9851.mov"  # Update with actual path
cap = cv2.VideoCapture(video_path)

# Background subtractor
fgbg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=False)

# Optical flow parameters
prev_frame = None
lk_params = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# List to store movement history
trajectory = []
smoothed_trajectory = []
alpha = 0.7  # Smoothing factor

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Resize for performance optimization
    frame = cv2.resize(frame, (640, 480))
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Initialize previous frame for optical flow
    if prev_frame is None:
        prev_frame = gray
        continue
    
    # Compute optical flow using Farneback method
    flow = cv2.calcOpticalFlowFarneback(prev_frame, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    
    # Compute motion magnitude
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    
    # Threshold motion to detect moving object
    motion_mask = cv2.inRange(mag, 1.0, 100.0)  # Adjust thresholds as needed
    
    # Find contours of motion mask
    contours, _ = cv2.findContours(motion_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        
        # Discard bounding boxes smaller than 50x50 pixels or non-square
        if w < 10 or h < 10 or abs(w - h) > 20:
            continue
        
        if cv2.contourArea(cnt) > 400:  # Adjust size threshold as needed
            center_x, center_y = x + w // 2, y + h // 2
            
            # Apply exponential smoothing to reduce jitter
            if smoothed_trajectory:
                prev_x, prev_y = smoothed_trajectory[-1]
                center_x = int(alpha * prev_x + (1 - alpha) * center_x)
                center_y = int(alpha * prev_y + (1 - alpha) * center_y)
            
            # Store position in trajectory list
            trajectory.append((center_x, center_y))
            smoothed_trajectory.append((center_x, center_y))
            
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Draw red dot at center
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
    
    # Draw trajectory path with velocity gradient (fixed width)
    thickness = 3  # Set fixed thickness
    for i in range(1, len(smoothed_trajectory)):
        if smoothed_trajectory[i - 1] is None or smoothed_trajectory[i] is None:
            continue
        
        # Calculate velocity as distance between points
        v = np.linalg.norm(np.array(smoothed_trajectory[i]) - np.array(smoothed_trajectory[i - 1]))
        
        # Normalize velocity to range for color mapping
        velocity_norm = min(max(int(v * 10), 0), 255)
        
        # Define color gradient (blue to red)
        color = (255 - velocity_norm, 0, velocity_norm)
        
        # Draw trajectory with fixed thickness
        cv2.line(frame, smoothed_trajectory[i - 1], smoothed_trajectory[i], color, thickness)
    
    # Display result
    cv2.imshow("Tracked Robot", frame)
    
    # Update previous frame
    prev_frame = gray.copy()
    
    if cv2.waitKey(30) & 0xFF == 27:  # Press 'ESC' to exit
        break

cap.release()
cv2.destroyAllWindows()
