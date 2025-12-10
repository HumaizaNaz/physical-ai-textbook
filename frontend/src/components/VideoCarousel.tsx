import React, { useState } from 'react';
import styles from './VideoCarousel.module.css'; // For component-specific styles
import clsx from 'clsx'; // For conditional class names

const videos = [
  '/videos/1.mp4',
  '/videos/2.mp4',
  '/videos/3.mp4',
];

export default function VideoCarousel() {
  const [currentVideoIndex, setCurrentVideoIndex] = useState(0);

  const goToNext = () => {
    setCurrentVideoIndex((prevIndex) => (prevIndex + 1) % videos.length);
  };

  const goToPrevious = () => {
    setCurrentVideoIndex((prevIndex) => (prevIndex - 1 + videos.length) % videos.length);
  };

  return (
    <div className={styles.carouselContainer}>
      <video
        key={videos[currentVideoIndex]} // Key prop to force re-render and reload video
        className={styles.videoPlayer}
        src={videos[currentVideoIndex]}
        controls
        autoPlay
        loop
        muted
      >
        Your browser does not support the video tag.
      </video>
      <div className={styles.controls}>
        <button onClick={goToPrevious} className={clsx(styles.controlButton, styles.left)}>
          &#10094; {/* Left arrow */}
        </button>
        <button onClick={goToNext} className={clsx(styles.controlButton, styles.right)}>
          &#10095; {/* Right arrow */}
        </button>
      </div>
      <div className={styles.dots}>
        {videos.map((_, index) => (
          <span
            key={index}
            className={clsx(styles.dot, { [styles.activeDot]: index === currentVideoIndex })}
            onClick={() => setCurrentVideoIndex(index)}
          ></span>
        ))}
      </div>
    </div>
  );
}
