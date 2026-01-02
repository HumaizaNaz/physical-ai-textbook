
   import path from 'path';
import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

// This runs in Node.js - Don't use client-side code here

const config: Config = {
   title: 'Physical AI & Humanoid Robotics',
  tagline: "Artificial Intelligence meets Embodied Cognition",
  favicon: "img/logo.png",

  future: {
    v4: true,
  },

  url: "https://physical-ai-textbook-five.vercel.app",
  baseUrl: "/",

  organizationName: "HumaizaNaz",
  projectName: "physical-ai-textbook",

  onBrokenLinks: "warn",

  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          editUrl:
            "https://github.com/HumaizaNaz/physical-ai-textbook/tree/main/frontend/",
        },
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: "img/docusaurus-social-card.jpg",
    colorMode: {
      defaultMode: "dark",
      respectPrefersColorScheme: true,
      disableSwitch: false,
    },
    navbar: {
      title: "Physical AI & Humanoid Robotics",
      logo: {
        alt: "My Site Logo",
        src: "img/logo.png",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Book",
        },
        {
          type: 'custom-AuthNavbarItem',
          position: 'right',
        },
        {
          href: "https://github.com/HumaizaNaz/physical-ai-textbook",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      links: [
        {
          title: "Docs",
          items: [
            {
              label: "Book",
              to: "/docs/category/introduction",
            },
          ],
        },
        {
          title: "Community",
          items: [
            {
              label: "Stack Overflow",
              href: "https://stackoverflow.com/questions/tagged/docusaurus",
            },
            {
              label: "Discord",
              href: "https://discordapp.com/invite/docusaurus",
            },
            {
              label: "X",
              href: "https://x.com/docusaurus",
            },
          ],
        },
        {
          title: "More",
          items: [
            {
              label: "GitHub",
              href: "https://github.com/HumaizaNaz/physical-ai-textbook",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,

  plugins: [
    [
      path.resolve(__dirname, "./plugins/api-rewrite-plugin.ts"),
      {
        backendUrl: process.env.NODE_ENV === 'development' ? "http://localhost:7860" : "https://better-auth-taupe-nine.vercel.app",
      },
    ],
    async function () {
      return {
        name: 'navbar-auth-injector',
        clientModules: [
          '@site/src/clientModules/navbarAuthInjector',
        ],
      };
    },
  ],
};

export default config;