# Complex Data Input

Some of the (sub-)behaviors in the Turtlesim demonstrations use operator input.

For this we provide a simple pop-up dialog window as part of the `flexbe_input` package in the `flexbe_behavior_engine` repository 
using the [`input_action_server`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_input/flexbe_input/input_action_server.py) .

This simple demonstration is intended to provide basic functionality for limited 
primitive inputs such as numbers or list/tuples of numbers.

You are encouraged to develop more complex user interfaces for more complex data structures as needed to support more advanced types.

The `InputState` makes use of the `pickle` module, and is subject to this warning from the Pickle manual:

>   Warning The pickle module is not secure against erroneous or maliciously constructed data. 
>   Never unpickle data received from an untrusted or unauthenticated source.

*If using the `InputState` it is up to the user to protect their network from untrusted data!*

ROS 2 messages support serializing using `pickle.dumps` and passing to the `InputState`.
For example, the code fragment below illustrates creating a `Pose` message to send to FlexBE in response to an action request.

```python
import ast
import pickle

from geometry_msgs.msg import Pose
from flexbe_msgs.action import BehaviorInput


# On custom design side (either UI or user developed action server node)
p = Pose()
p.position.x = 42.

result = BehaviorInput.Result()
result.data = str(pickle.dumps(p))  # format data for sending as string of bytes

# On input state side 
input_data = ast.literal_eval(result.data)  # convert string to byte array
response_data = pickle.loads(input_data)  # loads data into Python object

print(f" respose =?= original : {p == response_data}")  # validate conversion
```

You are invited to create your own variations of the [`input_action_server`](https://github.com/FlexBE/flexbe_behavior_engine/blob/ros2-devel/flexbe_input/flexbe_input/input_action_server.py) to handle more complex types.



