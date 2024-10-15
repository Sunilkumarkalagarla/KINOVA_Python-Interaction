# Install and Import the necessary Packages
from ultralytics import YOLO
import cv2

# Load the YOLO Model
model = YOLO('yolov8n.pt')

# Use laptop's camera (when it is an index value)
# If it used "rtsp://192.168.1.10/color", it is using the ip camera address (which is of robot's hand)
cap = cv2.VideoCapture(0)

while True:
    # ret is the boolean value that returns whether it read the frame or not
    # frame is the frame returned by the VideoCapture() function
    ret, frame = cap.read()

    if ret:
        # Detect and Track Objects
        results = model.track(frame, persist=True)

        # Get detection results
        detections = results[0].boxes.data.tolist()

        # Print detection results (class, confidence, bounding box)
        for detection in detections:
            class_id = int(detection[5])
            confidence = detection[4]
            bbox = detection[:4]
            print(f"Class: {model.names[class_id]}, Confidence: {confidence:.2f}, Bounding Box: {bbox}")

        # Plot the Results
        frame_ = results[0].plot()

        # Visualize the Results
        cv2.imshow('frame', frame_)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# To release stop the video streaming and object detection.
cap.release()
# Destroy all windows
cv2.destroyAllWindows()