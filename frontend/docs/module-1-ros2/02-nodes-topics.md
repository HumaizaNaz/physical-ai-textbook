---
sidebar_position: 2
title: ROS 2 Nodes & Topics
---

# ROS 2 Nodes & Topics: The Building Blocks of Communication

In the ROS 2 architecture, nodes and topics are fundamental for creating distributed robotic applications. They enable different parts of your robot's software to communicate and collaborate effectively.

## Understanding ROS 2 Nodes

A **Node** in ROS 2 is essentially an an executable process that performs a specific, single-purpose computation. Think of it as a module or a program responsible for a particular task.

**Examples of Nodes:**
*   A node reading data from a camera sensor.
*   A node controlling a robot's motor.
*   A navigation node calculating paths.
*   A user interface node displaying robot status.

Nodes are independent processes that can be written in various languages (Python, C++) and run on different machines. They communicate without needing to know each other's internal implementation details.

## Mastering ROS 2 Topics

**Topics** are the primary mechanism for asynchronous, one-to-many communication in ROS 2. They operate on a publish/subscribe model:

*   **Publishers**: Nodes that send messages to a specific topic.
*   **Subscribers**: Nodes that receive messages from a specific topic.

Messages are data structures that contain information. ROS 2 provides a wide range of standard message types (e.g., `sensor_msgs/msg/Image`, `geometry_msgs/msg/Twist`). You can also define custom message types.

### How Nodes Communicate via Topics

When a node publishes a message to a topic, all nodes subscribed to that same topic will receive a copy of the message. This decouples the senders from the receivers, making the system highly flexible and scalable.

**Example Scenario**:
Imagine a mobile robot.
1.  A "Camera Driver Node" publishes `Image` messages to the `/camera/image_raw` topic.
2.  A "Computer Vision Node" subscribes to `/camera/image_raw` to process images.
3.  A "Mapping Node" also subscribes to `/camera/image_raw` to build a map.

Neither the Camera Driver knows about the Computer Vision Node or Mapping Node, nor do they know about the Camera Driver. They only interact through the `/camera/image_raw` topic.

---

### Co-Learning Elements

#### 💡 Theory: Decoupled Design
The use of nodes and topics promotes a decoupled design paradigm in robotics. Each node can be developed, tested, and deployed independently. This modularity reduces complexity, improves maintainability, and allows for easier integration of new functionalities or hardware components into a robotic system.

#### 🎓 Key Insight: The Topic Graph
While seemingly simple, the collective set of all nodes and topics forms a dynamic "topic graph" at runtime. Understanding how data flows through this graph is crucial for debugging, optimizing, and visualizing the state of a complex robotic system. Tools like `rqt_graph` are invaluable for visualizing this live communication network.

#### 💬 Practice Exercise: Ask your AI

**Prompt**: "Generate a simple Python ROS 2 publisher and subscriber example for a humanoid robot. The publisher should send the robot's 'heartbeat' status (a boolean message indicating 'alive' or 'healthy') every second, and the subscriber should print the received status."

**Instructions**: Use your preferred AI assistant to generate two Python scripts: one `heartbeat_publisher.py` and one `heartbeat_subscriber.py`. Ensure they use a standard ROS 2 boolean message type and can run as ROS 2 nodes. Include basic ROS 2 setup (imports, node initialization, timer/loop, etc.).
```