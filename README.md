# Teleop

![Lite6 Physical Teleoperation](./media/lite6_physical_teleop.gif)

## Installation
```bash
pip3 install teleop
```

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
