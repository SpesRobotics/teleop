# Teleop


https://github.com/user-attachments/assets/9fa6a7fc-b2da-4970-9f46-9ff48b99a2fb


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

```bash
pip3 install -e .
```
