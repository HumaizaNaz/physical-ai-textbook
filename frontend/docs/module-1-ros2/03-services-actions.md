---
sidebar_position: 3
title: ROS 2 Services & Actions
---

# ROS 2 Services & Actions: Interactive and Goal-Oriented Communication

Beyond the basic publish/subscribe model of topics, ROS 2 provides more structured communication patterns for specific needs: Services for immediate request-response interactions, and Actions for managing complex, long-running tasks.

## ROS 2 Services: Request-Reply Communication

**Services** in ROS 2 enable a client node to send a request to a service-providing node and wait for a response. This is a synchronous, one-to-one communication pattern, ideal for queries that require an immediate result.

**Characteristics of Services**:
*   **Synchronous**: The client typically blocks until it receives a response.
*   **One-to-one**: One client communicates with one service server.
*   **Atomic**: Represents a single transaction, like a function call.

**Example Scenario**:
A humanoid robot's "Arm Control Node" might expose a service `/set_pose` that takes a target joint configuration as a request and returns `success/failure` as a response after moving the arm.

## ROS 2 Actions: Long-Running Goal Execution

**Actions** are designed for tasks that are long-running and goal-oriented. Unlike services, actions provide continuous feedback about the progress of the goal, and they can be preempted (cancelled) by the client.

**Characteristics of Actions**:
*   **Asynchronous**: The client doesn't block but receives feedback while the goal is being processed.
*   **Goal-oriented**: Defined by a specific goal, feedback during execution, and a final result.
*   **Preemptable**: Clients can cancel a goal that is in progress.

**Action Communication Flow**:
1.  **Goal**: Client sends a goal to the action server.
2.  **Feedback**: Action server sends periodic updates on progress to the client.
3.  **Result**: Action server sends the final result when the goal is completed or aborted.

**Example Scenario**:
A humanoid robot might use an action `/walk_to_target` where the goal is a `(x, y)` coordinate. The action server would provide feedback on the robot's current position, and the final result would be `reached_target` or `aborted`. The user could preempt this action if an obstacle is detected.

## Services vs. Actions: When to Use Which?

| Feature         | ROS 2 Service                       | ROS 2 Action                             |
|-----------------|-------------------------------------|------------------------------------------|
| **Communication** | Synchronous Request-Reply           | Asynchronous Goal-Feedback-Result        |
| **Duration**    | Short, immediate response           | Long-running tasks                       |
| **Feedback**    | None (only final response)          | Continuous progress feedback             |
| **Preemption**  | Not possible                        | Possible (client can cancel)             |
| **Use Cases**   | Querying state, setting single values | Navigation, manipulation, complex behaviors |

---

### Co-Learning Elements

#### 💡 Theory: State Machines for Robotic Tasks
Actions in ROS 2 naturally lend themselves to the implementation of state machines for complex robotic behaviors. The action goal, feedback, and result correspond directly to the states and transitions of a finite state machine, providing a structured way to manage the robot's progression through a task.

#### 🎓 Key Insight: The Necessity of Asynchronicity
While services offer simplicity for quick interactions, long-running tasks *demand* asynchronicity. Without actions, a client requesting a 30-second navigation task would be blocked for its entire duration, making the system unresponsive. Actions free the client to perform other operations while monitoring the long task's progress.

#### 💬 Practice Exercise: Ask your AI

**Prompt**: "Generate a simple Python ROS 2 action server and client example for a humanoid robot. The action should be to 'perform a wave gesture', with the server providing feedback on arm joint angles and the client receiving the final success status."

**Instructions**: Use your preferred AI assistant to create two Python scripts: one `wave_action_server.py` and one `wave_action_client.py`. Assume a custom action message type `Wave.action` with a boolean goal (e.g., `perform_wave`), float array feedback (e.g., `current_joint_angles`), and a boolean result (e.g., `wave_successful`).
```