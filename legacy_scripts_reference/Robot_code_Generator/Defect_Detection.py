import cv2
import numpy as np
from collections import deque

class PrecisionTracker:
    def __init__(self):
        self.kf = cv2.KalmanFilter(6, 3)
        self.kf.measurementMatrix = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0]], np.float32)
        
        self.kf.transitionMatrix = np.array([
            [1,0,0,1,0,0],
            [0,1,0,0,1,0],
            [0,0,1,0,0,1],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]], np.float32)
        
        self.kf.processNoiseCov = 1e-4 * np.eye(6, dtype=np.float32)
        self.kf.measurementNoiseCov = 1e-2 * np.eye(3, dtype=np.float32)
        
        self.measurements = deque(maxlen=5)
        self.confidence = 0
        self.best_radius = 0

    def update(self, measurement):
        if measurement is not None:
            x, y, r = measurement
            self.measurements.append((x, y, r))
            self.confidence = min(10, self.confidence + 2)
            
            # Use median of recent measurements
            med_x = np.median([m[0] for m in self.measurements])
            med_y = np.median([m[1] for m in self.measurements])
            med_r = np.median([m[2] for m in self.measurements])
            
            self.best_radius = med_r
            self.kf.correct(np.array([[np.float32(med_x)], 
                                     [np.float32(med_y)], 
                                     [np.float32(med_r)]]))
        else:
            self.confidence = max(0, self.confidence - 1)
        
        return self.kf.predict()

def enhance_circle_visibility(frame):
    """Improved preprocessing specifically for circle detection"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Adaptive histogram equalization
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    equalized = clahe.apply(gray)
    
    # Dynamic bilateral filtering
    blur_size = max(3, int(0.02 * min(frame.shape[:2])))
    if blur_size % 2 == 0: blur_size += 1
    blurred = cv2.bilateralFilter(equalized, blur_size, 75, 75)
    
    return blurred

def find_circles(image, tracker=None):
    """Robust circle detection with validation"""
    circles = cv2.HoughCircles(
        image,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=50,
        param1=50,
        param2=30,
        minRadius=10,
        maxRadius=300
    )
    
    if circles is None:
        return None
    
    circles = np.uint16(np.around(circles))
    valid_circles = []
    
    for (x, y, r) in circles[0,:]:
        # Create circular mask
        mask = np.zeros_like(image)
        cv2.circle(mask, (x, y), r, 255, -1)
        
        # Calculate contrast at circle boundary
        masked = cv2.bitwise_and(image, image, mask=mask)
        outer_ring = cv2.bitwise_and(image, image, mask=cv2.subtract(
            cv2.dilate(mask, np.ones((5,5), np.uint8)), mask))
        
        mean_inner = masked[mask > 0].mean()
        mean_outer = outer_ring[outer_ring > 0].mean()
        contrast = abs(mean_inner - mean_outer)
        
        # Validate circle quality
        if contrast > 25:  # Minimum contrast threshold
            if tracker is not None and tracker.confidence > 3:
                # When tracking, verify against expected radius
                if 0.7 * tracker.best_radius < r < 1.3 * tracker.best_radius:
                    valid_circles.append((x, y, r, contrast))
            else:
                valid_circles.append((x, y, r, contrast))
    
    if not valid_circles:
        return None
    
    # Select best circle by contrast and size consistency
    valid_circles.sort(key=lambda c: (-c[3], -c[2]))  # Sort by contrast then size
    return valid_circles[0][:3]  # Return (x, y, r)

def main():
    tracker = PrecisionTracker()
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        frame = cv2.flip(frame, 1)
        processed = enhance_circle_visibility(frame)
        
        # First pass detection
        circle = find_circles(processed, tracker)
        
        # Refinement if needed
        if circle is None and tracker.confidence > 3:
            # If tracking but lost detection, try focused search
            x, y = int(tracker.kf.statePost[0]), int(tracker.kf.statePost[1])
            r = int(tracker.best_radius)
            roi_size = r * 3
            roi = processed[max(0,y-roi_size):min(frame.shape[0],y+roi_size),
                          max(0,x-roi_size):min(frame.shape[1],x+roi_size)]
            if roi.size > 0:
                local_circle = find_circles(roi)
                if local_circle is not None:
                    circle = (local_circle[0]+x-roi_size, 
                             local_circle[1]+y-roi_size, 
                             local_circle[2])
        
        # Update tracker
        prediction = tracker.update(circle)
        
        # Visualization
        if tracker.confidence > 3 and prediction is not None:
            x, y = int(prediction[0]), int(prediction[1])
            r = int(prediction[2])
            
            # Color based on confidence
            color = (0, min(255, tracker.confidence * 25), 255 - min(255, tracker.confidence * 25))
            
            # Draw circle and center
            cv2.circle(frame, (x, y), r, color, 2)
            cv2.circle(frame, (x, y), 2, color, 3)
            
            # Draw crosshair
            cv2.line(frame, (x-15, y), (x+15, y), color, 1)
            cv2.line(frame, (x, y-15), (x, y+15), color, 1)
            
            # Display info
            info_text = f"({x},{y}) R:{r} C:{tracker.confidence}"
            cv2.putText(frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 1)
        
        cv2.imshow('Precision Circle Detection', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()