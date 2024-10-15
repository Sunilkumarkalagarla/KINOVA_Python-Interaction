# Import necessary packages
from ultralytics import YOLO
import cv2

# Load the YOLO Model
model = YOLO('yolov8n.pt')

# Load Video (For Now)
video_path = './test.mp4' # Edit the Path with the actual video path (of your choice)
cap = cv2.VideoCapture(video_path)

ret = True

while ret:
    ret, frame = cap.read()

    if ret:
        # Detect and Track Objects
        results = model.track(frame, persist = True)
        # Plot the Results
        frame_ = results[0].plot()

        # Visualize the Results
        cv2.imshow('frame', frame_)
        if cv2.waitkey(25) & 0xFF == ord('q'):
            break