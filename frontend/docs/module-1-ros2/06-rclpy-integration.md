---
sidebar_position: 1
title: rclpy Integration
---

# rclpy Integration: Bringing ROS 2 to Python

`rclpy` is the official Python client library for ROS 2. It provides a straightforward and intuitive way to write ROS 2 nodes, allowing Python developers to leverage the full power of the ROS 2 ecosystem for robotics applications. Its ease of use makes it a popular choice for prototyping and high-level control logic.

## Creating Your First ROS 2 Node in Python

Every ROS 2 application starts with a node. In `rclpy`, creating a node involves inheriting from `rclpy.node.Node` and initializing the superclass.

```python
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_custom_node')
        self.get_logger().info('My Custom Node has been started!')

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    my_node = MyCustomNode() # Create the node
    rclpy.spin(my_node) # Keep node alive
    my_node.destroy_node() # Destroy node when done
    rclpy.shutdown() # Shut down rclpy

if __name__ == '__main__':
    main()
```

## Implementing Publishers and Subscribers

`rclpy` makes it easy to create publishers and subscribers to communicate via topics.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Example message type

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing Service Clients and Servers

Services facilitate a request-response pattern.

### Service Server Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Example service type

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)
        self.get_logger().info('Request sent')

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    minimal_client.send_request()
    
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().error('Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (minimal_client.req.a, minimal_client.req.b, response.sum))
            break
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Co-Learning Elements

#### 💡 Theory: The Event Loop
`rclpy.spin()` is central to how ROS 2 Python nodes process events. It essentially puts the node into an infinite loop, allowing it to listen for incoming messages, service requests, action goals, and timer callbacks. This "event loop" is crucial for managing asynchronous operations and ensuring the node remains responsive to the ROS 2 graph.

#### 🎓 Key Insight: Pythonic ROS 2 Development
`rclpy` is designed to be idiomatic Python, making it accessible to developers familiar with the language. It leverages Python's strengths for rapid prototyping and scripting, allowing for quicker development cycles, especially for higher-level control logic and user interfaces, complementing `rclcpp` (C++ client library) for performance-critical components.

#### 💬 Practice Exercise: Ask your AI

**Prompt**: "Generate a Python `rclpy` node that publishes a custom `JointState` message (from `sensor_msgs.msg`) for a simple one-joint robot. The node should publish the joint's position, velocity, and effort every 0.1 seconds, simulating real-time sensor data."

**Instructions**: Use your preferred AI assistant to write an `rclpy` node. Define a timer callback that populates a `JointState` message with arbitrary values for `name`, `position`, `velocity`, and `effort`, and publishes it to a `/joint_states` topic. Ensure proper `rclpy` initialization and shutdown.
```
}
```
