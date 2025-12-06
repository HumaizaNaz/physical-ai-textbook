---
sidebar_position: 2
---

# 02 ROS 2 Nodes, Topics, and Services

## 💡 Theory

ROS 2's architecture is built around a graph of communicating executable processes called **nodes**. Each node is responsible for a single, modular function (e.g., a camera driver, a motor controller, a path planner). Nodes communicate with each other primarily through **topics**, which are named buses for streaming data. A node can *publish* messages to a topic, and other nodes can *subscribe* to that topic to receive those messages. This asynchronous, many-to-many communication pattern is ideal for continuous data streams like sensor readings or motor commands. For request/reply interactions, ROS 2 provides **services**. A service allows a *client* node to send a request to a *server* node and wait for a response. This synchronous communication is suitable for operations like querying a robot's state or triggering a specific action that provides immediate feedback.

### ROS 2 Communication Patterns

| Pattern   | Communication Type | Use Case                                     | Characteristics                             |
| :-------- | :----------------- | :------------------------------------------- | :------------------------------------------ |
| **Topics**| Asynchronous, Many-to-Many | Sensor data, continuous state updates        | Real-time streams, non-blocking             |
| **Services**| Synchronous, One-to-One  | Request/reply, querying state, triggering actions | Blocking, immediate feedback expected       |

```python
# File: ros2_publisher_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# --- Publisher Node ---
class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.status_count = 0
        self.get_logger().info('SimplePublisher node started.')

    def publish_status(self):
        msg = String()
        msg.data = f'Robot is operational. Update: {self.status_count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.status_count += 1

# --- Subscriber Node ---
class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('SimpleSubscriber node started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main_publisher(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

def main_subscriber(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # To run, open two terminals:
    # Terminal 1: python -c "from ros2_publisher_subscriber import main_publisher; main_publisher()"
    # Terminal 2: python -c "from ros2_publisher_subscriber import main_subscriber; main_subscriber()"
    print("ROS 2 Node example. Run publisher and subscriber in separate terminals after sourcing ROS 2 environment.")
```

## 🎓 Key Insight

The choice between topics and services profoundly impacts the design and robustness of a robotic system. Topics are ideal for continuous, high-frequency data streams where the loss of an occasional message is acceptable (e.g., camera feeds, LiDAR scans). They enable highly concurrent operations as publishers don't wait for subscribers. Services, on the other hand, are critical for reliable, single-shot interactions where a response is expected (e.g., commanding a robot to move to a specific joint angle and waiting for confirmation). Misusing these patterns can lead to performance bottlenecks (e.g., using a service for high-frequency sensor data) or unreliable behavior (e.g., using a topic for critical commands that require acknowledgment). Understanding the strengths of each allows architects to design scalable and efficient communication graphs for humanoid robots.

```python
# File: ros2_service_example.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Standard ROS 2 service example

# --- Service Server Node ---
class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('MinimalService node started. Ready to add two ints.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: {response.sum}')
        return response

# --- Service Client Node ---
class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main_service_server(args=None):
    rclpy.init(args=args)
    node = MinimalService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

def main_service_client(args=None):
    rclpy.init(args=args)
    node = MinimalClientAsync()
    try:
        response = node.send_request(2, 3)
        node.get_logger().info(f'Result of add_two_ints: for 2 + 3 = {response.sum}')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # To run, open two terminals:
    # Terminal 1: python -c "from ros2_service_example import main_service_server; main_service_server()"
    # Terminal 2: python -c "from ros2_service_example import main_service_client; main_service_client()"
    print("ROS 2 Service example. Run server and client in separate terminals after sourcing ROS 2 environment.")
```

## 💬 Practice Exercise: "Ask your AI"

Design a ROS 2 communication strategy for a humanoid robot performing a 'pick-and-place' task. Specifically, how would you use nodes, topics, and services to: 1) get real-time joint positions (from motors), 2) send a command to open/close the gripper, and 3) request a complex inverse kinematics solution for a target object pose? Explain your choices. Provide a hypothetical `curl` command to the `/ros2-nodes-topics-services` endpoint that reports a snapshot of key ROS 2 communication activities, including recent topic messages and service call statistics.

```bash
# Live curl example for the FastAPI backend
# Assume FastAPI is running on http://localhost:8000
curl -X GET "http://localhost:8000/ros2-nodes-topics-services"
```

**Expected JSON Response (hypothetical, for ROS 2 communication snapshot):**
```json
{
  "status": "COMMUNICATION_ACTIVE",
  "robot_name": "Optimus-H1",
  "active_nodes": [
    "/joint_state_publisher",
    "/gripper_controller",
    "/ik_solver_service"
  ],
  "topic_activity": [
    {
      "topic": "/joint_states",
      "type": "sensor_msgs/msg/JointState",
      "last_message": "{ 'positions': [0.1, 0.2, ...], 'velocities': [...] }",
      "rate_hz": 50
    }
  ],
  "service_activity": [
    {
      "service": "/gripper/command",
      "type": "std_srvs/srv/SetBool",
      "last_request": "{ 'data': True }",
      "last_response": "{ 'success': True }",
      "calls_per_minute": 5
    },
    {
      "service": "/ik_solve",
      "type": "robot_msgs/srv/SolveIK",
      "last_request_timestamp": "2025-12-05T16:30:15Z",
      "last_response_timestamp": "2025-12-05T16:30:17Z",
      "calls_per_minute": 2
    }
  ],
  "timestamp": "2025-12-05T16:30:20Z"
}
```
