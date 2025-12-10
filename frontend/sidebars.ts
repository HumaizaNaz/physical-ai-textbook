import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: '01 Introduction to Physical AI',
      link: { type: 'doc', id: 'introduction/index' },
      items: [
        'introduction/foundations-of-physical-ai',
        'introduction/embodied-intelligence',
        'introduction/sensor-systems',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: '02 The Robotic Nervous System (ROS 2)',
      link: { type: 'doc', id: 'ros2-nervous-system/index' },
      items: [
        'ros2-nervous-system/ros2-fundamentals',
        'ros2-nervous-system/ros2-nodes-topics-services',
        'ros2-nervous-system/ros2-actions-parameters',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: '03 The Digital Twin',
      link: { type: 'doc', id: 'digital-twin/index' },
      items: [
        'digital-twin/urdf-xacro',
        'digital-twin/gazebo-simulation',
        'digital-twin/unity-robotics',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: '04 The AI-Robot Brain (NVIDIA Isaac)',
      link: { type: 'doc', id: 'isaac-brain/index' },                // ← fixed
      items: [
        'isaac-brain/isaac-sim-overview',                          // ← fixed
        'isaac-brain/synthetic-data-generation',                   // ← fixed
        'isaac-brain/perception-and-manipulation',                 // ← fixed
        'isaac-brain/reinforcement-learning-in-isaac-lab',         // ← fixed
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: '05 Vision-Language-Action & Capstone',
      link: { type: 'doc', id: 'vla-capstone/index' },               // ← fixed
      items: [
        'vla-capstone/vla-fundamentals',                           // ← fixed
        'vla-capstone/voice-to-action-pipeline',                   // ← fixed
        'vla-capstone/openvla-rtx-and-helix',                      // ← fixed
        'vla-capstone/full-vla-stack-integration',                // ← fixed
        'vla-capstone/capstone-autonomous-humanoid', 
        'vla-capstone/conversational-robotics',              // ← fixed
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;