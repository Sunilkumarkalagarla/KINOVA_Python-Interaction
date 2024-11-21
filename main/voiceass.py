import os
import sys
import threading

import pyttsx3
import speech_recognition as sr
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

import utilities
# import color
import cv2
import numpy as np

# import imageCapture as ic
# from gripper_close import close_gripper


# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 30

"""
        A Helper Function to monitor robot actions and signals an event when an action is finished (either successfully or aborted).
 This helps the program know when to proceed to the next step.
"""
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

def cartesian_action_movement(base, base_cyclic, action_name):
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    if action_name == "pick":
        feedback.base.tool_pose_y += 0.35 # (meters)
    elif action_name == "drop":
        feedback.base.tool_pose_z -= 0.15 # (meters)
    elif action_name == "up":
        feedback.base.tool_pose_z += 0.15 # (meters)
    elif action_name == "rest":
        feedback.base.tool_pose_z -= 0.045 # (meters)
    elif action_name == "right":
        feedback.base.tool_pose_theta_z += 10
    elif action_name == "left":
        feedback.base.tool_pose_theta_z -= 10
    elif action_name == "front":
        feedback.base.tool_pose_y += 0.1
    elif action_name == "back":
        feedback.base.tool_pose_y -= 0.1
    elif action_name == "go_left":
        feedback.base.tool_pose_y -= 0.05
    elif action_name == "go_right":
        feedback.base.tool_pose_y += 0.05
    elif action_name == "go_up":
        feedback.base.tool_pose_z += 0.05
    elif action_name == "go_down":
        feedback.base.tool_pose_z -= 0.0
    elif action_name == "go_back":
        feedback.base.tool_pose_x -= 0.05
    elif action_name == "go_forward":
        feedback.base.tool_pose_x += 0.05
    elif action_name == "turn_left":
        feedback.base.tool_pose_theta_z -= 90
    elif action_name == "turn_right":
        feedback.base.tool_pose_theta_z += 90
    elif action_name == "turn_around":
        feedback.base.tool_pose_theta_z -= 180




    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x  # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z   # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x  # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y  # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z  # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def move_to_a_position(base, position):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle_1 = None
    action_handle_2 = None
    for action in action_list.action_list:
        if action.name == position:
            action_handle_1 = action.handle

    if action_handle_1 == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle_1)


    # Leave time to action to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if not finished:
        print("Timeout on action notification wait")
    return finished

def open_gripper(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.SEND_GRIPPER_COMMAND
    action_list = base.ReadAllActions(action_type)
    action_handle_1 = None
    action_handle_2 = None
    for action in action_list.action_list:
        if action.name == "open_gripper":
            action_handle_1 = action.handle
        # print(action.name)
        # print(action.handle)

    if action_handle_1 == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle_1)

    # Leave time to action to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if not finished:
        print("Timeout on action notification wait")
    return finished


def gripper_close(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.SEND_GRIPPER_COMMAND
    action_list = base.ReadAllActions(action_type)
    action_handle_1 = None
    action_handle_2 = None
    for action in action_list.action_list:
        if action.name == "water_gripper_hold":
            action_handle_1 = action.handle
        # print(action.name)
        # print(action.handle)

    if action_handle_1 == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle_1)

    # Leave time to action to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if not finished:
        print("Timeout on action notification wait")
    return finished

def gripper_close_new(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.SEND_GRIPPER_COMMAND
    action_list = base.ReadAllActions(action_type)
    action_handle_1 = None
    action_handle_2 = None
    for action in action_list.action_list:
        if action.name == "newobject":
            action_handle_1 = action.handle
        # print(action.name)
        # print(action.handle)

    if action_handle_1 == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle_1)

    # Leave time to action to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if not finished:
        print("Timeout on action notification wait")
    return finished

def listen(timeout_duration=5):
    # Create an instance of the Recognizer class
    recognizer = sr.Recognizer()

    try:
        with sr.Microphone() as mic:
            recognizer.adjust_for_ambient_noise(mic, duration=0.2)
            print("Listening...")

            # Set a timeout for listening
            audio = recognizer.listen(mic, timeout=timeout_duration)

            # Convert speech to text
            text = recognizer.recognize_google(audio)
            print("You said: " + text)
            return text

    except sr.UnknownValueError:
        print("Sorry, I did not understand that.")
        return None

    except sr.RequestError as e:
        print(f"Could not request results from Google Speech Recognition service; {e}")
        return None

    except sr.WaitTimeoutError:
        print("Listening timed out while waiting for phrase to start.")
        return None

def pick_object(i):
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    # Parse arguments
    args = utilities.parseConnectionArguments()
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router, utilities.DeviceConnection.createUdpConnection(
            args) as router_real_time:
        # Create required services
        base = BaseClient(router)
        success = True
        if i:
            success &= move_to_a_position(base, "Bottle1_Top")
            success &= move_to_a_position(base, "Bottle1_Hold_Pos")
            success &= gripper_close(base)
            success &= move_to_a_position(base, "Bottle1_Top")
        elif i == 2:
            success &= move_to_a_position(base, "Bottle2_Top")
            success &= move_to_a_position(base, "Bottle2_Hold_Pos")
            success &= gripper_close(base)
            success &= move_to_a_position(base, "Bottle2_Top")
        elif i == 3:
            success &= move_to_a_position(base, "Bottle3_Top")
            success &= move_to_a_position(base, "Bottle3_Hold_Pos")
            success &= gripper_close(base)
            success &= move_to_a_position(base, "Bottle3_Top")
        success &= move_to_a_position(base, "Home")
        success &= move_to_a_position(base, "Rest")
        success &= open_gripper(base)

def speak_text(text):
    # Initialize the text-to-speech engine
    engine = pyttsx3.init()

    # Say the text
    engine.say(text)

    # Wait for the speech to complete
    engine.runAndWait()
    return None

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
def get_the_color(color_code):
    # Access the internal camera with ID 1
    # Use IP camera ("rtsp://192.168.1.10/color")
    cap = cv2.VideoCapture(1)
    print("I'm in the Color Code")

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
        if most_common_color == color_code:
            return True
        else:
            return False
    else:
        print("No color detected.")

def main():
    # pick_object(1)
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    # Parse arguments
    args = utilities.parseConnectionArguments()
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router,utilities.DeviceConnection.createUdpConnection(
            args) as router_real_time:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        # speak_text("What do you want me to do?")
        success = True
        while True:
            # str1 = listen()
            command1 = input("What do you want me to do now?: ")
            if command1 == 'go left':
                success &= cartesian_action_movement(base, base_cyclic, "go_left")
            elif command1 == 'go right':
                success &= cartesian_action_movement(base, base_cyclic, "go_right")
            elif command1 == 'go up':
                success &= cartesian_action_movement(base, base_cyclic, "go_up")
            elif command1 == 'go down':
                success &= cartesian_action_movement(base, base_cyclic, "go_down")
            elif command1 == 'turn left':
                success &= cartesian_action_movement(base, base_cyclic, "turn_left")
            elif command1 == 'turn right':
                success &= cartesian_action_movement(base, base_cyclic, "turn_right")
            elif command1 == 'go_forward':
                success &= cartesian_action_movement(base, base_cyclic, "go_forward")
            elif command1 == 'go back':
                success &= cartesian_action_movement(base, base_cyclic, "go_back")
            elif command1 == 'go home':
                speak_text("Going Home")
                success &= move_to_a_position(base, "Home")
            elif command1 == 'take rest':
                speak_text("Going to rest position")
                success &= move_to_a_position(base, "Rest")
            elif command1 == 'turn around':
                success &= cartesian_action_movement(base, base_cyclic, "turn_around")
            elif command1 == 'hold object':
                success &= gripper_close_new(base)
            elif command1 == 'stop':
                speak_text("Thank you Very much!")
                success &= move_to_a_position(base, "Rest")
                break
            elif command1 == 'pick up':
                # sampleEnter = int(input("Enter a number for fun: "))
                # pick_object(sampleEnter)
                # speak_text("Sure, Starting Pickup Routine")
                # vis_pos_str = int(input("Which product should I pick up? (1 or 2 or 3):"))
                success &= move_to_a_position(base, "Home")
                success &= open_gripper(base)
                position = ["Bottle1_Watch_Pos", "Bottle2_Watch_Pos", "Bottle3_Watch_Pos"]
                success = True
                color_code = input("Which color code would you like to pickup?: ")
                for i in range(3):
                    success &= move_to_a_position(base, position[i])
                    if get_the_color(color_code):
                        pick_object(i)
                        break
                else:
                    speak_text("Please Check is you have that color or it's my camera's fault!")
            elif command1 == 'open gripper' or 'drop':
                success &= open_gripper(base)
            elif command1 == 'capture image':
                speak_text("Please hold the image for 5 seconds")
                # ic.capture_image(0)
    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
