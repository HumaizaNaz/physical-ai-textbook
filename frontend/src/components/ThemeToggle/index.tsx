import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useColorMode } from '@docusaurus/theme-common';
import clsx from 'clsx';

import styles from './styles.module.css';
import { JSX } from 'react';

interface ThemeToggleProps {
  className?: string;
}

export default function ThemeToggle({ className }: ThemeToggleProps): JSX.Element {
  const { isDarkTheme, setColorMode } = useColorMode();
  const {
    siteConfig: { title },
  } = useDocusaurusContext();

  const toggleTheme = () => {
    setColorMode(isDarkTheme ? 'light' : 'dark');
  };

  return (
    <button
      type="button"
      aria-label={`Switch between ${isDarkTheme ? 'light' : 'dark'} mode`}
      className={clsx(
        'clean-btn',
        'navbar-sidebar__toggle', // Reuse Docusaurus toggle styling for basic layout
        styles.themeToggle,
        className,
      )}
      onClick={toggleTheme}>
      <span className={styles.iconContainer}>
        {isDarkTheme ? (
          <span className={clsx(styles.moonIcon, styles.orbitingRing)}></span> // Moon with orbiting ring
        ) : (
          <span className={styles.sunIcon}></span> // Sun
        )}
      </span>
    </button>
  );
}
