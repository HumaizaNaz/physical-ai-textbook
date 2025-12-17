# Task Example: Robotics Code Generator

## Context
This document provides a concrete example of how the `robotics-code-generator` Claude skill would be invoked and what its expected output would be. This demonstrates the skill's capability to generate production-ready Python code for Physical AI tasks.

## Skill Invocation
The `robotics-code-generator` skill would be invoked with a clear description of the desired robotics task.

### Prompt Example
```
Generate Python code for a ROS 2 Iron node that publishes a "Twist" message to the `/cmd_vel` topic to make a differential drive robot move forward at 0.1 m/s and turn left at 0.05 rad/s. Include proper ROS 2 imports, a node class, and error handling for shutdown.
```

## Expected Output
A markdown code block containing a fully commented, runnable Python script.

### Example Output Structure
````markdown
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time # For simulation of movement duration

class SimpleVelocityPublisher(Node):
    def __init__(self):
        super().__init__('simple_velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = None
        self.get_logger().info('SimpleVelocityPublisher node has been started.')

    def publish_velocity_command(self, linear_x, angular_z, duration_s=None):
        """
        Publishes a Twist message to cmd_vel topic.
        :param linear_x: Linear velocity in x-direction (m/s)
        :param angular_z: Angular velocity around z-axis (rad/s)
        :param duration_s: Optional duration in seconds to publish the command.
        """
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        self.get_logger().info(f'Publishing: Linear.x={msg.linear.x}, Angular.z={msg.angular.z}')
        self.publisher_.publish(msg)

        if duration_s:
            # For demonstration, continuously publish for duration
            end_time = time.time() + duration_s
            while time.time() < end_time and rclpy.ok():
                self.publisher_.publish(msg)
                time.sleep(0.1) # Publish at ~10 Hz for stability

    def shutdown_hook(self):
        """
        Stop the robot and clean up on shutdown.
        """
        self.get_logger().info('Shutting down: Sending stop command...')
        stop_msg = Twist() # All zeros
        self.publisher_.publish(stop_msg)
        self.get_logger().info('Stop command sent.')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleVelocityPublisher()

    try:
        # Example: Move forward and turn left for 5 seconds
        node.publish_velocity_command(0.1, 0.05, duration_s=5)
    except KeyboardInterrupt:
        pass # Handle Ctrl+C gracefully
    except Exception as e:
        node.get_logger().error(f"An unexpected error occurred: {e}")
    finally:
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
````
