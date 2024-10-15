import cv2
import os
import time

# Opens the camera of Robot's Hand to capture video.
# Use "rtsp://192.168.1.10/color" to connect to the robot's camera
cap = cv2.VideoCapture(1)

i = 0

while(cap.isOpened() and i < 2):
        ret, frame = cap.read()
        if not ret:
                break
        path = r'/Users/sunilkumarkalagarla/PycharmProjects/KINOVA_Python/main/images'
        cv2.imwrite(os.path.join(path, 'frame' + str(i) + '.jpg'), frame)
        i += 1

cap.release()
cv2.destroyAllWindows()