import cv2
import numpy as np

# Define a function to get the dominant color in a given region of interest (ROI)
def get_dominant_color(hsv_roi):
    # Calculate the mean of each channel in the HSV space
    mean_hue = np.mean(hsv_roi[:, :, 0])
    mean_saturation = np.mean(hsv_roi[:, :, 1])
    mean_value = np.mean(hsv_roi[:, :, 2])

    # Determine the color based on the hue
    if mean_saturation > 50 and mean_value > 50:
        if 0 <= mean_hue < 15 or 165 < mean_hue <= 180:
            color = "Red"
        elif 15 <= mean_hue < 35:
            color = "Yellow"
        elif 35 <= mean_hue < 85:
            color = "Green"
        elif 85 <= mean_hue < 125:
            color = "Blue"
        elif 125 <= mean_hue < 165:
            color = "Purple"
        else:
            color = "Unknown"
    else:
        color = "Gray or Black (Low Saturation/Value)"

    return color

# Access the internal camera with ID 1
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Parameters for capturing multiple frames
num_frames = 10  # Number of frames to capture
colors_detected = []

for _ in range(num_frames):
    # Capture a single frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        continue

    # Define the region of interest (ROI) in the center of the frame
    height, width, _ = frame.shape
    cx, cy = width // 2, height // 2
    roi_size = 50  # Size of the central area
    roi = frame[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]

    # Convert the ROI to HSV color space
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Get the dominant color in the ROI and add it to the list
    dominant_color = get_dominant_color(hsv_roi)
    colors_detected.append(dominant_color)

# Release the camera after capturing the frames
cap.release()

# Calculate the most frequently detected color
if colors_detected:
    most_common_color = max(set(colors_detected), key=colors_detected.count)
    print("Detected color (most common across frames):", most_common_color)
    # ?return most_common_color
else:
    print("No color detected.")