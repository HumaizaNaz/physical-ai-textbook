---
sidebar_position: 2
title: URDF and SDF Robot Description Formats
---

# URDF and SDF Robot Description Formats: Defining Your Digital Robot

Accurately describing your robot is fundamental for both simulation and real-world deployment. In the ROS 2 ecosystem, two primary XML-based formats are used for this purpose: Unified Robot Description Format (URDF) and Simulation Description Format (SDF). While URDF is focused on the kinematic and dynamic description of a single robot, SDF is more comprehensive, capable of describing entire worlds, including multiple robots and static environmental elements.

## Unified Robot Description Format (URDF)

As introduced in Module 1, URDF is an XML format for describing the kinematic and dynamic properties of a robot. Its primary focus is on a single robot's structure.

**Key Features of URDF**:
*   **Links**: Represent the rigid bodies of the robot (e.g., base, arm segments, head). Each link can have associated mass, inertia, visual, and collision properties.
*   **Joints**: Define the connections between links, specifying the type of motion (e.g., revolute, prismatic, fixed) and their limits.
*   **Kinematic Tree**: URDF models typically form a single, acyclic kinematic tree, meaning a parent link can have multiple child links, but a child link can only have one parent.
*   **ROS 2 Integration**: Directly supported by ROS 2 tools for visualization (RViz), motion planning (MoveIt), and robot state publishing (`robot_state_publisher`).

**Example (simplified)**:
```xml
<robot name="simple_arm">
  <link name="base_link"/>
  <joint name="base_to_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="shoulder_link"/>
</robot>
```

## Simulation Description Format (SDF)

SDF is a more extensive XML format used primarily by Gazebo (and other simulators) to describe everything in a simulation world: robots, environments, objects, lights, and even plugins. SDF is designed to be a superset of URDF in terms of capabilities, able to describe complex scenes that URDF cannot.

**Key Features of SDF**:
*   **World Description**: Can describe an entire `world`, including `models` (robots, objects), `lights`, `sensors`, and `physics` parameters.
*   **Graph Structure**: Supports arbitrary graph structures, including closed kinematic chains (loops), which URDF does not.
*   **Full Physics Properties**: Provides more detailed and nuanced physics parameters (e.g., coefficients of friction, damping, restitution) compared to URDF.
*   **Plugins**: Allows definition of plugins at various levels (world, model, sensor) to extend functionality.
*   **Simulation-specific**: Designed for use in simulation environments, providing all necessary information for a simulator to run.

**Example (simplified)**:
```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="my_simple_world">
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
    </light>
    <model name="my_robot">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="base_link">
        <!-- ... link properties ... -->
      </link>
      <!-- ... joints and other links ... -->
    </model>
  </world>
</sdf>
```

## URDF vs. SDF: When to Use Which?

| Feature             | URDF                                     | SDF                                                  |
|:--------------------|:-----------------------------------------|:-----------------------------------------------------|
| **Primary Purpose** | Robot description for ROS 2 tools        | Full world description for simulators (e.g., Gazebo) |
| **Scope**           | Single robot                             | Multiple robots, environments, lights, sensors       |
| **Kinematics**      | Acyclic tree                             | Arbitrary graph (including closed loops)             |
| **Physics Detail**  | Basic (mass, inertia, collision)         | Comprehensive (friction, damping, full contact)      |
| **Flexibility**     | Less flexible, single robot focus        | More flexible, full simulation scene                 |

**General Guideline**: Often, a robot is described initially in URDF (or Xacro for modularity), and then this URDF is converted or embedded into an SDF file if the robot needs to be simulated in Gazebo. The `ros_gz_bridge` (or similar) can then manage communication between the ROS 2 control stack and the Gazebo simulation.

---

### Co-Learning Elements

#### 💡 Theory: The Digital Twin
Both URDF and SDF contribute to the creation of a "digital twin"—a virtual replica of a physical system. For robotics, a digital twin allows for real-time monitoring, analysis, and control of the physical counterpart, enabling predictive maintenance, remote operation, and advanced simulation for AI development.

#### 🎓 Key Insight: The Importance of Consistent Modeling
Maintaining consistency between your robot's URDF and SDF descriptions is paramount. Discrepancies in joint limits, inertial properties, or collision meshes between the two formats can lead to significant "reality gaps" where robot behavior in simulation does not accurately reflect reality, hindering effective development and testing.

#### 💬 Practice Exercise: Ask your AI

**Prompt**: "Explain how a ROS 2 robot described in URDF can be used within a Gazebo simulation that primarily uses SDF. What tools or mechanisms are typically used to bridge these two description formats?"

**Instructions**: Use your preferred AI assistant to describe the process, mentioning the role of `gazebo_ros_pkgs` or similar bridges, and how URDF models are often incorporated into SDF worlds.
```