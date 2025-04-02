import cv2
import numpy as np

# Load video
video_path = "IMG_9851.mov"  # Update with actual path
cap = cv2.VideoCapture(video_path)

# Parameters for Lucas-Kanade Optical Flow
lk_params = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Shi-Tomasi corner detection parameters
feature_params = dict(maxCorners=1, qualityLevel=0.3, minDistance=7, blockSize=7)

# Read the first frame
ret, old_frame = cap.read()
if not ret:
    print("Error: Could not read video")
    cap.release()
    exit()

gray_old = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(gray_old, mask=None, **feature_params)

# Create a mask for drawing trajectory
mask = np.zeros_like(old_frame)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Calculate optical flow
    if p0 is not None:
        p1, st, err = cv2.calcOpticalFlowPyrLK(gray_old, gray_frame, p0, None, **lk_params)
        
        # Select good points
        if p1 is not None and st[0][0] == 1:
            a, b = p1.ravel()
            c, d = p0.ravel()
            
            # Draw trajectory
            mask = cv2.line(mask, (int(c), int(d)), (int(a), int(b)), (0, 255, 0), 2)
            frame = cv2.add(frame, mask)
            
            # Update for next iteration
            p0 = p1.reshape(-1, 1, 2)
    
    gray_old = gray_frame.copy()
    
    # Display the tracking
    cv2.imshow("Optical Flow Tracking", frame)
    
    if cv2.waitKey(30) & 0xFF == 27:  # Press 'ESC' to exit
        break

cap.release()
cv2.destroyAllWindows()
