---
sidebar_position: 4
title: Building ROS 2 Packages with Python
---

# Building ROS 2 Packages with Python: Organizing Your Robotic Code

In ROS 2, a **package** is the fundamental unit for organizing software. It contains source code, build scripts, configuration files, and other resources related to a specific piece of functionality (e.g., a robot driver, a navigation module). For Python-based ROS 2 development, `ament_python` provides the necessary tools and conventions for creating, building, and installing packages.

## Understanding ROS 2 Packages

A ROS 2 package typically consists of:

*   **`package.xml`**: Defines metadata about the package, including its name, version, description, maintainers, license, build dependencies, and runtime dependencies.
*   **`setup.py`**: The standard Python build script for `ament_python` packages. It specifies how Python modules and scripts within the package should be installed.
*   **`resource` folder**: Contains a marker file (e.g., `share/<package_name>/package.xml`) that helps ROS 2 find the package.
*   **`src` folder**: Contains your Python source code.
*   **`launch` folder**: (Optional) Contains Python or XML launch files to start multiple ROS 2 nodes.
*   **`config` folder**: (Optional) Contains YAML configuration files for parameters.

## Creating a New Python Package

You can create a new ROS 2 Python package using the `ros2 pkg create` command:

```bash
ros2 pkg create --build-type ament_python my_python_package --dependencies rclpy std_msgs
```

**Explanation**:
*   `--build-type ament_python`: Specifies that this is a Python package built with `ament`.
*   `my_python_package`: The name of your new package.
*   `--dependencies rclpy std_msgs`: Automatically adds `rclpy` (the Python client library for ROS 2) and `std_msgs` (standard message types) as dependencies to `package.xml` and `setup.py`.

This command will create the basic directory structure and populate `package.xml` and `setup.py`.

## The `package.xml` File

The `package.xml` is crucial for ROS 2 package management. Key tags include:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_python_package</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## The `setup.py` File

The `setup.py` script defines how your Python code is structured and what executables should be exposed as ROS 2 nodes.

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_python_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # Example for launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_python_package.publisher_member_function:main', # Example node executable
            'listener = my_python_package.subscriber_member_function:main',
        ],
    },
)

```

**Explanation of `entry_points`**: This section maps a console script name (e.g., `talker`) to a Python function within your package (e.g., `my_python_package.publisher_member_function:main`). This allows you to run your ROS 2 nodes directly using `ros2 run my_python_package talker`.

## Building and Installing Your Package

After creating your package, you need to build and install it using `colcon`:

```bash
# From the root of your ROS 2 workspace (e.g., ~/ros2_ws)
colcon build --packages-select my_python_package
```

Then, source your workspace to make the new package available:

```bash
source install/setup.bash # or setup.zsh, setup.ps1
```

Now you can run your nodes: `ros2 run my_python_package talker`.

---

### Co-Learning Elements

#### 💡 Theory: The `ament` Build System
`ament` (Ament Build System) is the meta-build system used in ROS 2. It orchestrates the building of various package types (including `ament_python`, `ament_cmake`) and ensures dependencies are correctly handled across different programming languages and build tools. It's an evolution of ROS 1's `catkin` build system, designed for greater flexibility and modularity.

#### 🎓 Key Insight: Separation of Concerns
ROS 2 package structure inherently promotes separation of concerns. By encapsulating specific functionalities within packages, developers can create modular, reusable components. This modularity simplifies development, testing, and maintenance, especially in large-scale robotics projects with many interacting parts.

#### 💬 Practice Exercise: Ask your AI

**Prompt**: "Generate a complete `package.xml` and `setup.py` for a new ROS 2 Python package called `humanoid_description_publisher`. This package should depend on `rclpy` and `sensor_msgs`. It should also expose a console script named `joint_state_publisher` that runs a Python function `main` from a module `joint_state_publisher_node` within the package."

**Instructions**: Use your preferred AI assistant to provide the full `package.xml` and `setup.py` file contents. Ensure all necessary metadata and entry points are correctly defined.
```