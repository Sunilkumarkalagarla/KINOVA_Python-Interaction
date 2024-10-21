from ultralytics import YOLO
import cv2
import math

# Load the YOLO Model
model = YOLO('yolov8n.pt')

# Use IP camera ("rtsp://192.168.1.10/color")
# o or 1 uses laptop's camera
cap = cv2.VideoCapture("rtsp://192.168.1.10/color")

# ---  Distance Estimation Parameters ---
KNOWN_WIDTH = 15  # Example: 15 cm
FOCAL_LENGTH = 1000  # Example: 1000 pixels

def calculate_distance(known_width, focal_length, pixel_width):
    """
    Calculates the distance to an object based on its known width,
    the camera's focal length, and the object's width in pixels.
    """
    return (known_width * focal_length) / pixel_width

# List to store unique detected objects
detected_objects = []

while True:
    ret, frame = cap.read()

    if ret:
        results = model.track(frame, persist=True)
        detections = results[0].boxes.data.tolist()

        for detection in detections:
            class_id = int(detection[5])
            class_name = model.names[class_id]
            confidence = detection[4]
            bbox = detection[:4]

            xmin, ymin, xmax, ymax = map(int, bbox)
            pixel_width = xmax - xmin
            distance = calculate_distance(KNOWN_WIDTH, FOCAL_LENGTH, pixel_width)

            # Check if the object is already in the list
            is_unique = True
            for obj in detected_objects:
                if obj["class"] == class_name:
                    is_unique = False
                    break

            # Add the object to the list if it's unique
            if is_unique:
                detected_objects.append({
                    "class": class_name,
                    "confidence": confidence,
                    "bbox": bbox,
                    "distance": distance
                })

        frame_ = results[0].plot()
        cv2.imshow('frame', frame_)

        key = cv2.waitKey(1)
        if key == 27:  # ASCII Value of ESC key
            break

# Print the list of unique detected objects
print("Detected Objects:")
for obj in detected_objects:
    print(obj["class"])
cap.release()
cv2.destroyAllWindows()