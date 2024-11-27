# Image Conversion Node (ROS 2)

## Overview

This ROS 2 package implements a node for converting images from a camera (published via the `usb_cam` package) to either grayscale or color. The node subscribes to the camera image topic, hosts a service to toggle between color and grayscale modes, and publishes the processed image on a new topic.

## Dependencies

- ROS 2 Humble
- usb_cam package (for USB camera input)
- OpenCV (for image processing)

## Installation Instructions

1. **Clone the repository:**

    ```bash
    cd ~/ros2_humble_ws/src
    git clone https://github.com/munavirzaman-git/image_conversion.git
    ```

2. **Install dependencies:**

    Ensure the `usb_cam` package is installed by running the following:

    ```bash
    sudo apt install ros-humble-usb-cam
    ```

3. **Build the workspace:**

    Navigate to the workspace and build the package:

    ```bash
    cd ~/ros2_humble_ws
    colcon build
    source install/setup.bash
    ```

## Running the Package

1. **Start the nodes using the launch file:**

    Use the following command to launch both the `usb_cam` node (which publishes the camera stream) and the `image_conversion` node (which processes the images):

    ```bash
    ros2 launch image_conversion image_conversion_launch.py
    ```

    This will start the `usb_cam` node on the default camera topic and the `image_conversion` node which will process and publish the images on the specified output topic.

2. **Service for Mode Change:**

    The `image_conversion` node hosts a ROS 2 service that allows the user to toggle between grayscale and color modes. You can call the service as follows:

    - **Grayscale Mode (Mode 1):**

      To change the mode to grayscale, use the following command:

      ```bash
      ros2 service call /image_conversion/set_mode std_srvs/srv/SetBool "{data: true}"
      ```

    - **Color Mode (Mode 2):**

      To change the mode back to color, use the following command:

      ```bash
      ros2 service call /image_conversion/set_mode std_srvs/srv/SetBool "{data: false}"
      ```

    The node will now either process the images in grayscale (Mode 1) or publish them without conversion (Mode 2).

## Parameters

You can change the following parameters in the `image_conversion_launch.py` launch file:

- **input_camera_topic**: The topic to which the camera images are being published (default is `/usb_cam/image_raw`).
- **output_image_topic**: The topic where the processed images will be published (default is `/image_conversion/image_processed`).

## Troubleshooting

- If the camera is not showing up, ensure that your USB camera is properly connected and recognized by your system.
- Make sure all dependencies are installed correctly and that the ROS 2 workspace is built without errors.

## License

This package is licensed under the MIT License.

## Author

Munavir Zaman

