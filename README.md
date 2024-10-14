# Image Converter C++ (ROS2)

`image_converter_cpp` is a ROS2 package that subscribes to a color image topic, converts the image to grayscale, and publishes it to a new topic. The input and output topic names are configurable via a YAML file.

## Features
- Subscribes to a color image topic (`raw_image` by default).
- Converts the color image to grayscale using OpenCV.
- Publishes the grayscale image to a new topic (`mono_image` by default).
- Uses a YAML configuration file to set input and output topic names.

## Requirements
- ROS2 (compatible with ROS2 Dashing or later)
- OpenCV
- YAML-CPP
- cv_bridge

## Installation

1. Ensure you have a ROS2 workspace set up:

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

2. Clone the repository into your workspace:

    ```bash
    cd ~/ros2_ws/src
    git clone <repository-url> image_converter_cpp
    ```

3. Install dependencies:

    ```bash
    sudo apt update
    sudo apt install ros-<your-ros2-distro>-cv-bridge
    sudo apt install ros-<your-ros2-distro>-yaml-cpp-vendor
    ```

4. Build the package:

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select image_converter_cpp
    ```

5. Source your workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Usage

1. Create a YAML configuration file to specify the input and output topics:

    Example: `config/image_converter_config.yaml`

    ```yaml
    topics:
      input_topic: "raw_image"
      output_topic: "mono_image"
    ```

2. Run the node, passing the YAML configuration file as an argument:

    ```bash
    ros2 run image_converter_cpp image_converter /absolute/path/to/image_converter_config.yaml
    ```

3. The node will subscribe to the `raw_image` topic (or the input topic specified in the YAML file), convert the color image to grayscale, and publish it to the `mono_image` topic (or the output topic specified in the YAML file).

## Example

### Example YAML configuration

```yaml
topics:
  input_topic: "/camera/color/image_raw"
  output_topic: "/camera/mono/image_raw"
```

### Example command to run the node:

```bash
ros2 run image_converter_cpp image_converter /home/ubuntu/ros2_ws/src/image_converter_cpp/config/image_converter_config.yaml
```

## Troubleshooting

- Ensure that the path to the YAML file is correct and accessible.
- If you encounter a `YAML::BadFile` error, verify that the YAML file exists and has the correct permissions.
- Check the ROS2 topics using `ros2 topic list` to confirm the input and output topics are being correctly published and subscribed to.

## License

This project is licensed under the Apache-2.0 license.