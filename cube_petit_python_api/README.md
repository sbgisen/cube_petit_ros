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
import rclpy
from commanders.speech import SpeechCommander
rclpy.init()
node = Node('test')
speech_node = SpeechCommander(node)
speech_node.say("test")
```

### Usage:Use Cupe-petit Chat via GPT-4
Need speech_to_text  `/julius_talk_result`
Topic can publish like `ros2 topic pub --once /julius_talk_result std_msgs/msg/String "data: 'こんにちは'"`
Need env `OPENAI_API_KEY`
Add param `setting_file`: path to setting file

```.py
from rclpy.node import Node
import rclpy
from cube_petit_python_api.commanders.gpt_chat import GPTChatCommander
rclpy.init()
node = Node('test')
gpt_chat_node = GPTChatCommander(node)
gpt_chat_node.chat("こんにちは")
```

### Usage:Use just GPT-4
Need env `OPENAI_API_KEY`
Add param `setting_file`: path to setting file

```.py
from rclpy.node import Node
import rclpy
from cube_petit_python_api.utils.gpt_client import GPTClient
rclpy.init()
node = Node('test')
gpt_node = GPTClient(self, api_key=`api_key`)
gpt_node.get_response(こんにちは)
```
