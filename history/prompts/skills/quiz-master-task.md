# Task Example: Quiz Master

## Context
This document provides a concrete example of how the `quiz-master` Claude skill would be invoked and what its expected output would be. This demonstrates the skill's capability to create high-quality multiple-choice questions (MCQs) with full explanations.

## Skill Invocation
The `quiz-master` skill would be invoked with content from which to generate quiz questions.

### Prompt Example
```
Generate 4 MCQs based on the following content about ROS 2 basics:

"ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It consists of a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors. At its core, ROS 2 enables communication between different processes (nodes) running on a robot, which can be distributed across multiple machines. These nodes communicate by passing messages over topics, which are named buses for data. For example, a camera node might publish image messages on an 'image_raw' topic, and a vision processing node might subscribe to that topic. Services provide a request/response mechanism, useful for actions that require a direct answer, like asking a robot to perform a specific task once. Actions are similar to services but are designed for long-running tasks that provide continuous feedback and can be preempted. Parameters allow nodes to be configured dynamically without recompiling the code."
```

## Expected Output
Exactly 4 high-quality MCQs, each with a question, four choices, the correct answer marked, and a detailed explanation.

### Example Output Structure
````markdown
**Question 1:** What is the primary function of "nodes" in ROS 2?
A. To store persistent data
B. To provide a request/response mechanism
C. To communicate by passing messages over topics
D. To represent individual processes running on a robot
**Correct Answer: D**
**Explanation:** Nodes are individual processes in ROS 2, and they communicate with each other. While they do pass messages, their primary identity is as a distinct process or executable unit.

**Question 2:** Which ROS 2 communication mechanism is best suited for continuous feedback during a long-running task that can be canceled?
A. Topics
B. Services
C. Actions
D. Parameters
**Correct Answer: C**
**Explanation:** Actions are specifically designed for long-running, goal-oriented tasks that require continuous feedback and can be preempted or cancelled, making them ideal for complex robot behaviors like navigation.

**Question 3:** A sensor publishing data streams (e.g., camera images) in ROS 2 would primarily use which communication method?
A. Services
B. Actions
C. Parameters
D. Topics
**Correct Answer: D**
**Explanation:** Topics are used for asynchronous, one-way streaming of data, making them perfect for sensors that continuously publish information like camera images, lidar scans, or IMU readings.

**Question 4:** How can a ROS 2 node be configured dynamically without needing to recompile its source code?
A. By modifying its actions
B. By adjusting its parameters
C. By changing its services
D. By subscribing to a new topic
**Correct Answer: B**
**Explanation:** Parameters in ROS 2 allow for the dynamic configuration of node behavior at runtime. This means values can be changed without requiring the node to be stopped, recompiled, and relaunched.
````
