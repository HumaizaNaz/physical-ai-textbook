import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      link: {
        type: 'generated-index',
        title: 'Introduction to Physical AI',
        description: 'Foundations of Physical AI and embodied intelligence for humanoid robotics.',
        slug: '/category/introduction',
      },
      items: [
        'introduction/foundations-of-physical-ai',
        'introduction/embodied-intelligence',
        'introduction/sensor-systems',
        'introduction/hardware-requirements',
        'introduction/sim-to-real-tips',
        'introduction/ethical-hri-safety',
      ],
    },
    {
      type: 'category',
      label: 'The Robotic Nervous System (ROS 2)',
      link: {
        type: 'generated-index',
        title: 'ROS 2 Overview',
        description: 'Learn the fundamentals of ROS 2 for robotic control and communication.',
        slug: '/category/module-1-ros2',
      },
      items: [
        'module-1-ros2/ros2-architecture',
        'module-1-ros2/nodes-topics',
        'module-1-ros2/services-actions',
        'module-1-ros2/rclpy-integration',
        'module-1-ros2/urdf-for-humanoids',
        'module-1-ros2/building-ros2-packages-python',
        'module-1-ros2/launch-files-parameters',
      ],
    },
    {
      type: 'category',
      label: 'The Digital Twin (Gazebo & Unity)',
      link: {
        type: 'generated-index',
        title: 'Digital Twin Overview',
        description: 'Explore robotics simulation with Gazebo and high-fidelity visualization with Unity.',
        slug: '/category/module-2-digital-twin',
      },
      items: [
        'module-2-digital-twin/gazebo-setup',
        'module-2-digital-twin/physics-sensor-simulation',
        'module-2-digital-twin/unity-visualization',
        'module-2-digital-twin/urdf-sdf-formats',
        'module-2-digital-twin/physics-sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'The AI-Robot Brain (NVIDIA Isaac™)',
      link: {
        type: 'generated-index',
        title: 'NVIDIA Isaac™ Ecosystem',
        description: 'Delve into NVIDIA Isaac Sim, Isaac ROS, and AI techniques for robotics.',
        slug: '/category/module-3-isaac',
      },
      items: [
        'module-3-isaac/isaac-sdk-isaac-sim',
        'module-3-isaac/ai-perception-manipulation',
        'module-3-isaac/reinforcement-learning-robot-control',
        'module-3-isaac/navigation-nav2',
        'module-3-isaac/isaac-sdk-isaac-sim',
        'module-3-isaac/ai-perception-manipulation',
        'module-3-isaac/reinforcement-learning-robot-control',
        'module-3-isaac/sim-to-real-transfer',
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA)',
      link: {
        type: 'generated-index',
        title: 'VLA and Capstone',
        description: 'Integrate vision, language, and action for advanced humanoid robot control.',
        slug: '/category/module-4-vla',
      },
      items: [
        'module-4-vla/llm-ros-action-planner',
        'module-4-vla/humanoid-kinematics-dynamics',
        'module-4-vla/bipedal-locomotion-balance',
        'module-4-vla/manipulation-grasping',
        'module-4-vla/natural-human-robot-interaction',
        'module-4-vla/gpt-conversational-ai',
        'module-4-vla/speech-recognition-nlu',
        'module-4-vla/multi-modal-interaction',
        'module-4-vla/voice-to-action-whisper',
        'module-4-vla/cognitive-planning-llms',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      link: {
        type: 'generated-index',
        title: 'Capstone Project',
        description: 'Bringing it all together: End-to-End Robot Pipeline.',
        slug: '/category/capstone',
      },
      items: [
        'capstone/capstone-project-overview',
        'capstone/capstone-rubric',
      ],
    },
    {
      type: 'category',
      label: 'Assessments',
      link: {
        type: 'generated-index',
        title: 'Assessments Overview',
        description: 'Overview of assessment components for the textbook.',
        slug: '/category/assessments',
      },
      items: [
        'assessments/assessments-overview',
      ],
    },
  ],
};

export default sidebars;
