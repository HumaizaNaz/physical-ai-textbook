---
sidebar_position: 3
---

# 03 Sensor Systems

## 💡 Theory

Sensor systems are the 'eyes' and 'ears' of a robot, providing the crucial data that enables it to perceive its environment and its own internal state. These systems transform physical phenomena (like light, sound, distance, force) into electrical signals that the robot's AI can process. Robots utilize a diverse array of sensors, each with specific strengths and weaknesses, to build a comprehensive understanding of their surroundings. Integrating multiple sensor modalities (e.g., cameras for vision, LiDAR for depth, IMUs for orientation) allows for robust perception, compensating for the limitations of individual sensors and providing richer contextual information. Effective sensor fusion, the process of combining data from various sensors, is paramount for accurate and reliable robotic operation in complex, real-world environments.

### Common Robot Sensor Types

| Sensor Type        | Modality        | Primary Use Case                          | Advantages                 | Disadvantages                  |
| :----------------- | :-------------- | :---------------------------------------- | :------------------------- | :----------------------------- |
| **Camera**         | Vision (Light)  | Object recognition, scene understanding   | Rich visual data           | Lighting sensitive, compute-intensive |
| **LiDAR**          | Ranging (Light) | 3D mapping, obstacle detection            | Accurate depth, 360° scan  | Expensive, susceptible to rain/fog |
| **IMU**            | Inertial        | Orientation, acceleration, angular velocity | Real-time pose estimation  | Drift over time, no position |
| **Ultrasonic**     | Ranging (Sound) | Proximity detection, simple obstacle avoidance | Low cost, robust to light  | Low resolution, environmental interference |
| **Force/Torque**   | Contact         | Grasping, manipulation, collision detection | Direct physical interaction| Limited range, specialized   |

```python
# File: sensor_simulation.py
import numpy as np

class SensorSimulator:
    def __init__(self, noise_level=0.1):
        self.noise_level = noise_level

    def simulate_lidar(self, distance_to_obstacle):
        """Simulates a LiDAR reading with noise."""
        noise = np.random.normal(0, self.noise_level)
        return distance_to_obstacle + noise

    def simulate_imu(self, true_orientation):
        """Simulates an IMU pitch reading with noise."""
        noise = np.random.normal(0, self.noise_level / 5)
        return true_orientation + noise

# Example usage:
simulator = SensorSimulator(noise_level=0.05)

obstacle_dist = 2.5 # meters
lidar_reading = simulator.simulate_lidar(obstacle_dist)
print(f"Simulated LiDAR reading (true: {obstacle_dist:.2f}m): {lidar_reading:.2f}m")

true_pitch = 0.1 # radians
imu_reading = simulator.simulate_imu(true_pitch)
print(f"Simulated IMU pitch (true: {true_pitch:.2f} rad): {imu_reading:.2f} rad")
```

## 🎓 Key Insight

Reliable perception is the bedrock of robust physical AI. A robot's ability to accurately sense its environment directly impacts the quality of its decision-making and the safety of its operations. Challenges in sensor systems include noise, calibration errors, sensor degradation, and environmental interference (e.g., glare for cameras, soft materials for LiDAR). Advanced sensor fusion algorithms, often employing probabilistic methods like Kalman filters or particle filters, are used to combine redundant and complementary information from multiple sensors, providing a more accurate and stable state estimate than any single sensor could provide alone. This fused perception is then fed into the AI's cognitive modules for planning and control.

```python
# File: sensor_fusion_concept.py
# This conceptual code demonstrates a simple weighted sensor fusion idea.
# In reality, complex probabilistic filters (e.g., Kalman) are used.

def simple_sensor_fusion(lidar_reading, ultrasonic_reading, lidar_weight=0.7, ultrasonic_weight=0.3):
    """Combines two sensor readings with predefined weights."""
    fused_reading = (lidar_reading * lidar_weight) + (ultrasonic_reading * ultrasonic_weight)
    return fused_reading

# Example readings (LiDAR is typically more accurate, so higher weight)
lidar_dist_accurate = 1.05
ultrasonic_dist_noisy = 0.90

fused_distance = simple_sensor_fusion(lidar_dist_accurate, ultrasonic_dist_noisy)
print(f"Fused distance: {fused_distance:.2f}m")

# Another example with more discrepancy
lidar_dist_clear = 5.0
ultrasonic_dist_close = 0.3 # Ultrasonic might pick up a nearby phantom object

fused_distance_discrepant = simple_sensor_fusion(lidar_dist_clear, ultrasonic_dist_close)
print(f"Fused distance with discrepancy: {fused_distance_discrepant:.2f}m (LiDAR's higher weight dominates)")
```

## 💬 Practice Exercise: "Ask your AI"

Consider a humanoid robot designed to operate in a busy hospital environment. It needs to navigate crowded hallways, identify patients and staff, and deliver medication. Based on this scenario, propose an optimal sensor suite, justifying your choices for each sensor type (e.g., LiDAR for mapping, thermal cameras for patient detection without privacy invasion). How would you implement sensor fusion to ensure reliable perception despite dynamic conditions and potential sensor failures? Provide a hypothetical `curl` command to a FastAPI endpoint that simulates a robot's current fused sensor data, including health status for each sensor, and describe the expected JSON response.

```bash
# Live curl example to get simulated sensor data from FastAPI backend
# Assume FastAPI is running on http://localhost:8000
curl -X GET "http://localhost:8000/sensor-data"
```

**Expected JSON Response (hypothetical, enriched with sensor health):**
```json
{
  "robot_id": "MediBot-001",
  "timestamp": "2025-12-05T15:45:00Z",
  "fused_environment_state": {
    "distance_to_nearest_obstacle": 0.75,
    "human_count": 5,
    "current_room": "Hallway_B",
    "ambient_light_lux": 450
  },
  "individual_sensor_status": {
    "lidar_front": {"health": "OK", "last_reading": 0.78, "error_rate": 0.01},
    "camera_rgbd": {"health": "OK", "last_reading_timestamp": "2025-12-05T15:44:59Z", "resolution": "1080p"},
    "ultrasonic_left": {"health": "WARNING", "last_reading": 0.25, "notes": "Possible dust interference"},
    "imu": {"health": "OK", "orientation": {"roll": 0.02, "pitch": 0.01, "yaw": 0.05}}
  },
  "operational_mode": "NAVIGATION"
}
```
