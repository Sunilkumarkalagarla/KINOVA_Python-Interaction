import time
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

# Constants
TIMEOUT_DURATION = 30

# Helper function to monitor robot action
def check_for_end_or_abort(event):
    def check(notification, e=event):
        print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event in [
            Base_pb2.ACTION_END,
            Base_pb2.ACTION_ABORT
        ]:
            e.set()
    return check

# Function to move the robot to a specified position
def move_to_position(base_client, position):
    """
    Moves the robot to the specified position.
    :param base_client: BaseClient object to communicate with the robot
    :param position: List containing [x, y, z] coordinates for the target position
    """
    action = Base_pb2.Action()
    action.name = "Move to position"
    action.application_data = ""

    actuator_count = base_client.GetActuatorCount().count
    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = position[0]
    cartesian_pose.y = position[1]
    cartesian_pose.z = position[2]
    cartesian_pose.theta_x = 90.0
    cartesian_pose.theta_y = 0.0
    cartesian_pose.theta_z = 90.0

    # Execute the action
    event = threading.Event()
    notification_handle = base_client.OnNotificationActionTopic(check_for_end_or_abort(event), Base_pb2.NotificationOptions())
    base_client.ExecuteAction(action)

    finished = event.wait(TIMEOUT_DURATION)
    base_client.Unsubscribe(notification_handle)

    if not finished:
        print("Timeout on action notification wait")
    else:
        print("Action completed successfully")

# Function to control the gripper
def control_gripper(base_client, close_gripper=True):
    """
    Controls the gripper to either close or open.
    :param base_client: BaseClient object to communicate with the robot
    :param close_gripper: Boolean indicating whether to close (True) or open (False) the gripper
    """
    gripper_command = Base_pb2.GripperCommand()
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    gripper_finger = gripper_command.gripper.finger.add()
    gripper_finger.value = 1.0 if close_gripper else 0.0

    base_client.SendGripperCommand(gripper_command)
    time.sleep(2)

# Main function to move, pick, and place an object
def move_pick_and_place(base_client, base_cyclic_client, start_position, target_position):
    """
    Moves the robot to a start position, grips an object, and moves it to a target position.
    :param base_client: BaseClient object to communicate with the robot
    :param base_cyclic_client: BaseCyclicClient object for cyclic feedback
    :param start_position: List containing [x, y, z] coordinates for the start position
    :param target_position: List containing [x, y, z] coordinates for the target position
    """
    # Move to start position
    print("Moving to start position...")
    move_to_position(base_client, start_position)

    # Close gripper to pick up the object
    print("Gripping the object...")
    control_gripper(base_client, close_gripper=True)

    # Move to target position
    print("Moving to target position...")
    move_to_position(base_client, target_position)

    # Open gripper to release the object
    print("Releasing the object...")
    control_gripper(base_client, close_gripper=False)

# Example usage
if __name__ == "__main__":
    import utilities
    import threading

    # Create connection to the robot
    parser = utilities.DeviceConnection.createArgumentParser() if hasattr(utilities.DeviceConnection, 'createArgumentParser') else utilities.parseArguments()
    args = parser.parse_args()
    router = utilities.DeviceConnection.createTcpConnection(args)
    router.connect()

    base_client = BaseClient(router)
    base_cyclic_client = BaseCyclicClient(router)

    # Define start and target positions
    start_position = [0.4, 0.0, 0.2]  # Example coordinates for picking
    target_position = [0.6, 0.2, 0.3]  # Example coordinates for placing

    try:
        # Perform the move, pick, and place operation
        move_pick_and_place(base_client, base_cyclic_client, start_position, target_position)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        router.disconnect()
