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
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: '01 Introduction to Physical AI',
      link: {type: 'doc', id: '01-introduction/index'},
      items: [
        '01-introduction/01-foundations-of-physical-ai',
        '01-introduction/02-embodied-intelligence',
        '01-introduction/03-sensor-systems',
      ],
    },
    {
      type: 'category',
      label: '02 The Robotic Nervous System (ROS 2)',
      link: {type: 'doc', id: '02-ros2-nervous-system/index'}, // Placeholder for future index.md
      items: [
        // Future ROS 2 lessons
      ],
    },
    {
      type: 'category',
      label: '03 The Digital Twin',
      link: {type: 'doc', id: '03-digital-twin/index'}, // Placeholder for future index.md
      items: [
        // Future Digital Twin lessons
      ],
    },
    {
      type: 'category',
      label: '04 The AI-Robot Brain (NVIDIA Isaac)',
      link: {type: 'doc', id: '04-isaac-brain/index'}, // Placeholder for future index.md
      items: [
        // Future Isaac Brain lessons
      ],
    },
    {
      type: 'category',
      label: '05 Vision-Language-Action & Capstone',
      link: {type: 'doc', id: '05-vla-capstone/index'}, // Placeholder for future index.md
      items: [
        // Future VLA & Capstone lessons
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
