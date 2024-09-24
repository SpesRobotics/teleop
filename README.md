# Teleop

Turns your phone into a robot arm teleoperation device by leveraging the WebXR API.

![Lite6 Physical Teleoperation](./media/lite6_physical_teleop.gif)

## Installation
```bash
pip3 install teleop
```

## Usage

### Basic Interface

A simple interface that simply prints the teleop response.

```bash
python3 -m teleop.basic
```

### ROS 2 Interface

A ROS 2 interface that publishes the teleop response to a topic.

```bash
python3 -m teleop.ros2
```

Published topics:
- `target_frame` (geometry_msgs/PoseStamped): The target pose of the robot arm end effector in the robot base frame.
- `tf` (tf2_msgs/TFMessage): The transform between the robot base frame and the target frame for visualization.

Subscribed topics:
- `current_pose` (geometry_msgs/PoseStamped): The current pose of the robot arm end effector in the robot base frame. Used for updating the reference pose.


You can use the standard ROS 2 arguments to override the default values, for example:
```bash
python3 -m teleop.ros2 --ros-args -r target_frame:=/some_other_topic_name
```

### Custom Interface
```python
from teleop import Teleop


def callback(pose, message):
    print(f'Pose: {pose}')
    print(f'Message: {message}')


teleop = Teleop()
teleop.subscribe(callback)
teleop.run()
```

## Development

### Install
```bash
pip3 install -e .
```

### Test

```bash
python3 -m pytest
```
