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
        'digital-twin/gazebo-simulation',
        'digital-twin/unity-robotics',
      ],
    },
    {
      type: 'category',
      label: '04 The AI-Robot Brain (NVIDIA Isaac)',
      link: { type: 'doc', id: 'intro' }, 
      items: [],
    },
    {
      type: 'category',
      label: '05 Vision-Language-Action & Capstone',
      link: { type: 'doc', id: 'intro' }, 
      items: [],
    },
  ],
};

export default sidebars;
