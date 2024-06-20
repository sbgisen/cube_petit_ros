## Build

```
colcon build
source ~/ros/install/setup.bash
```

Check if `ls /home/gisen/ros/install/cube_python_api/lib/python3.10/site-packages/cube_python_api` exist

```
$ ls /home/gisen/ros/install/cube_python_api/lib/python3.10/site-packages/cube_python_api
commanders  cube_commander.py  __init__.py  __pycache__  utils
```

### Usage:Speech

[Terminal1]
```
ros2 launch cube_speech cube_speech.launch.py
```

[Terminal2]
```.py
from rclpy.node import Node
import rcypy
from cube_petit_python_api.commanders.speech import SpeechCommander
rclpy.init()
node = Node('test')
speech_node = SpeechCommander(node)
speech_node.say("test")
```
