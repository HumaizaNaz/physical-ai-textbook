# Task Example: Lesson Builder

## Context
This document provides a concrete example of how the `lesson-builder` Claude skill would be invoked and what its expected output would be. This demonstrates the skill's capability to structure raw text into a comprehensive lesson plan.

## Skill Invocation
The `lesson-builder` skill would be invoked with raw, unstructured or semi-structured text content intended to form a lesson.

### Prompt Example
```
Structure the following raw text into a lesson plan:

"Physical AI involves intelligent agents that perceive, act, and learn in the physical world. This goes beyond purely software-based AI by integrating robotics and embodied intelligence. We'll explore ROS 2 as the communication backbone. For code, think about publishing simple 'Twist' messages. A good practice would be to simulate a robot moving forward. Summary: Physical AI merges AI with robotics, using ROS 2 for communication. Objectives: Define Physical AI, explain ROS 2 nodes, publish Twist message."
```

## Expected Output
A structured lesson plan including Title, Objectives, Concepts, Code, Practice, and Summary sections.

### Example Output Structure
````markdown
# Understanding Physical AI and ROS 2 Basics

## Objectives
*   Define what Physical AI is and its distinction from purely software AI.
*   Explain the role of ROS 2 as a communication backbone in Physical AI systems.
*   Demonstrate how to publish a `Twist` message for basic robot movement.

## Concepts
Physical AI refers to intelligent agents that are physically embodied and interact with the real world. This field uniquely blends artificial intelligence with robotics, where the agent's physical form and actions are integral to its intelligence and learning process. ROS 2 (Robot Operating System 2) serves as a critical middleware that enables various components (nodes) of a robot to communicate and coordinate their actions. It provides standardized ways to send and receive data (messages) over named channels (topics), issue commands, and configure behaviors.

## Code
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Robot Controller Node started. Publishing Twist messages.')

    def move_forward(self, speed=0.1):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = 0.0 # No rotation
        self.publisher_.publish(msg)
        self.get_logger().info(f'Moving forward at {speed} m/s.')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        node.move_forward()
        rclpy.spin(node) # Keep node alive
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practice
Simulate a robot moving forward using the provided ROS 2 code. Experiment with different `speed` values and observe the robot's simulated behavior.

## Summary
Physical AI integrates AI with robotics, allowing intelligent agents to interact with the physical world. ROS 2 is essential for communication within these systems. We covered how to define Physical AI, the role of ROS 2 nodes, and how to issue basic movement commands using `Twist` messages.
````
