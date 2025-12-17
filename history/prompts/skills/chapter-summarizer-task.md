# Task Example: Chapter Summarizer

## Context
This document provides a concrete example of how the `chapter-summarizer` Claude skill would be invoked and what its expected output would be. This demonstrates the skill's capability to summarize chapter content into bullet points with bold key terms.

## Skill Invocation
The `chapter-summarizer` skill would be invoked with the full text of a chapter or a significant section thereof.

### Prompt Example
```
Summarize the following chapter content:

"The first section introduces the fundamental principles of embodied intelligence, emphasizing that an agent's physical form and interaction with its environment are crucial for its cognitive development. Unlike traditional AI, which often focuses solely on abstract reasoning, embodied AI considers the tight coupling between perception, action, and cognition. For example, a robot learning to walk doesn't just compute gaits; it senses its balance, adapts to uneven terrain, and uses its physical body to explore and learn about its surroundings. This physical interaction shapes its understanding.

The second section delves into sensor systems, which are the primary means by which embodied agents perceive their environment. Common sensors include cameras for visual input, lidar for depth perception, force sensors for tactile feedback, and gyroscopes for orientation. The quality and type of sensor data directly influence the agent's ability to build an internal model of the world and make informed decisions. Advanced sensor fusion techniques are often employed to combine data from multiple modalities, providing a more robust and comprehensive understanding.

Finally, the chapter discusses the implications of these concepts for designing robust Physical AI systems. It highlights the importance of hardware-software co-design, where the physical capabilities of the robot are considered alongside its algorithmic intelligence. Future directions include developing more versatile and adaptive physical platforms capable of learning complex manipulation tasks in unstructured environments, drawing heavily from biological inspiration."
```

## Expected Output
A summary in 6-8 bullet points with bolded key terms.

### Example Output Structure
````markdown
*   **Embodied intelligence** posits that cognitive abilities stem from **physical form** and **environmental interaction**, differing from abstract-focused traditional AI.
*   **Perception, action, and cognition** are tightly coupled, influencing a robot's learning and understanding.
*   **Sensor systems** (cameras, lidar, force sensors, gyroscopes) are vital for **embodied agents** to perceive and build internal world models.
*   **Sensor fusion** techniques combine data from multiple **modalities** for robust environmental understanding.
*   **Physical AI system design** emphasizes **hardware-software co-design**, integrating physical capabilities with algorithmic intelligence.
*   Future **Physical AI** focuses on **versatile physical platforms** and **complex manipulation tasks** in **unstructured environments**, often inspired by biology.
````
