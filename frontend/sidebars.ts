import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: '01 Introduction to Physical AI',
      link: {type: 'doc', id: 'introduction/index'},
      items: [
        'introduction/foundations-of-physical-ai',
        'introduction/embodied-intelligence',
        'introduction/sensor-systems',
      ],
    },
    {
      type: 'category',
      label: '02 The Robotic Nervous System (ROS 2)',
      link: {type: 'doc', id: 'ros2-nervous-system/index'},
      items: [
        'ros2-nervous-system/ros2-fundamentals',
        'ros2-nervous-system/ros2-nodes-topics-services',
        'ros2-nervous-system/ros2-actions-parameters',
      ],
    },
    {
      type: 'category',
      label: '03 The Digital Twin',
      link: {type: 'doc', id: 'digital-twin/index'},
      items: [
        'digital-twin/urdf-xacro',
      ],
    },
    {
      type: 'category',
      label: '04 The AI-Robot Brain (NVIDIA Isaac)',
      link: {type: 'doc', id: '04-isaac-brain/index'},
      items: [
        '04-isaac-brain/01-isaac-sim-overview',
        '04-isaac-brain/02-synthetic-data-generation',
        '04-isaac-brain/03-perception-and-manipulation',
        '04-isaac-brain/04-reinforcement-learning-in-isaac-lab',
      ],
    },
    {
      type: 'category',
      label: '05 Vision-Language-Action & Capstone',
      link: {type: 'doc', id: '05-vla-capstone/index'},
      items: [
        '05-vla-capstone/01-vla-fundamentals',
        '05-vla-capstone/02-voice-to-action-pipeline',
        '05-vla-capstone/03-openvla-rtx-and-helix',
        '05-vla-capstone/04-full-vla-stack-integration',
        '05-vla-capstone/05-capstone-autonomous-humanoid',
      ],
    }
  ],
};

export default sidebars;
