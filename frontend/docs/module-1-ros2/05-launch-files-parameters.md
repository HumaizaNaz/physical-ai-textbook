---
sidebar_position: 5
title: Launch Files and Parameter Management
---

# Launch Files and Parameter Management: Orchestrating ROS 2 Applications

As ROS 2 applications grow in complexity, manually starting multiple nodes, configuring their parameters, and managing their relationships becomes cumbersome and error-prone. ROS 2 **Launch Files** provide a powerful mechanism to automate the startup and configuration of entire robotic systems. Coupled with ROS 2 **Parameters**, they offer flexible control over node behavior without recompilation.

## Understanding ROS 2 Launch Files

A Launch File is an XML or Python script that defines how to start and configure a collection of ROS 2 nodes and other processes. They allow you to:

*   **Launch Multiple Nodes**: Start several nodes simultaneously with a single command.
*   **Set Parameters**: Assign initial values to node parameters.
*   **Remap Topics**: Change the names of topics that nodes publish or subscribe to.
*   **Include Other Launch Files**: Create modular launch configurations.
*   **Conditional Launching**: Start nodes or processes only if certain conditions are met.

Launch files are typically stored in the `launch/` directory of a ROS 2 package.

### Example: Python Launch File

Python launch files are highly flexible as they allow programmatic control.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='motor_driver',
            name='robot_motors',
            output='screen',
            parameters=[
                {'left_wheel_radius': 0.1},
                {'right_wheel_radius': 0.1},
                {'max_speed': 1.5}
            ]
        ),
        Node(
            package='my_robot_sensors',
            executable='camera_publisher',
            name='robot_camera',
            output='screen',
            remappings=[
                ('/camera/image_raw', '/robot/camera_feed')
            ]
        ),
    ])
```

**To run this launch file**: `ros2 launch my_robot_package my_robot_system.launch.py`

## ROS 2 Parameters: Dynamic Configuration

Parameters are dynamic configuration values associated with ROS 2 nodes. They allow you to modify a node's behavior at runtime without needing to recompile its source code. Parameters can be:

*   **Declared**: A node must explicitly declare its parameters.
*   **Set/Get**: Values can be set or retrieved using the ROS 2 command-line tools or programmatically within other nodes.
*   **Loaded from YAML**: Parameters can be loaded from YAML files, typically via launch files.

### Declaring and Using Parameters (Python Example)

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('robot_name', 'HumanoidAlpha')

        # Get initial values
        max_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.get_logger().info(f'Initial Max Speed: {max_speed}, Robot Name: {robot_name}')

        # Register a callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_linear_speed':
                self.get_logger().info(f'Parameter "max_linear_speed" changed to: {param.value}')
            elif param.name == 'robot_name':
                self.get_logger().info(f'Parameter "robot_name" changed to: {param.value}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Loading Parameters from YAML (via Launch File)

You can define parameters in a YAML file:

```yaml
# config/robot_params.yaml
parameter_node:
  ros__parameters:
    max_linear_speed: 0.75
    robot_name: "HumanoidBeta"
```

And load them in a Python launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_package'), # Replace with your package name
        'config',
        'robot_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='parameter_node',
            name='my_param_node',
            output='screen',
            parameters=[config] # Load parameters from YAML
        ),
    ])
```

---

### Co-Learning Elements

#### 💡 Theory: Configuration as Code
Launch files and external parameter YAMLs embody the "Configuration as Code" principle. Instead of manually configuring systems, their setup is defined in version-controlled scripts. This ensures reproducibility, simplifies deployment, and allows for easier management of complex robotic systems across different environments.

#### 🎓 Key Insight: Dynamic Adaptability
ROS 2 parameters introduce a powerful layer of dynamic adaptability. A robot's behavior can be altered on-the-fly (e.g., changing navigation speed, sensor thresholds) without stopping and restarting nodes. This is invaluable for debugging, fine-tuning performance, and adapting to changing mission requirements in a real-world scenario.

#### 💬 Practice Exercise: Ask your AI

**Prompt**: "Generate a Python ROS 2 launch file that starts two nodes: a `camera_driver` from `sensor_package` and a `vision_processor` from `perception_package`. The `vision_processor` node should have a parameter `detection_threshold` set to `0.7` and its input topic `/image_raw` should be remapped to `/robot/camera_feed`."

**Instructions**: Use your preferred AI assistant to create a Python launch file. Ensure both nodes are launched, the parameter is set, and the remapping is applied.
```