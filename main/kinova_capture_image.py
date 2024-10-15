import cv2
import numpy as np
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
# from kortex_api.autogen.client_stubs.CameraClient import CameraClient
from kortex_api.autogen.messages import DeviceConfig_pb2
from kortex_api.RouterClient import RouterClientSendOptions

def capture_and_save_image(filename="captured_image.png"):
    """
    Captures an image from the Kinova robot's camera and saves it to a file.

    Args:
        filename (str): The name of the file to save the image to.
    """
    try:
        # Initialize the router and session
        router = RouterClientSendOptions()
        device_manager = DeviceManagerClient(routers)
        camera_client = CameraClient(router)

        # Get the camera device ID
        devices = device_manager.GetDeviceList(DeviceConfig_pb2.DeviceType.CAMERA)
        camera_device_id = devices.device_id[0]

        # Capture an image
        image = camera_client.GetImage(camera_device_id)

        # Convert the image to a format OpenCV can use
        np_image = cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)

        # Save the image
        cv2.imwrite(filename, np_image)
        print(f"Image captured and saved as {filename}")

    except Exception as e:
        print(f"Error capturing and saving image: {e}")

# Example usage:
capture_and_save_image()  # Captures and saves the image as "captured_image.png"