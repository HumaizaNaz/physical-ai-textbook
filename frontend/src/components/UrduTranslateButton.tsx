import React from 'react';
import clsx from 'clsx';
import '../css/UrduTranslate.css'; // Import for side effects

interface UrduTranslateButtonProps {
  onClick: () => void;
}

const UrduTranslateButton: React.FC<UrduTranslateButtonProps> = ({ onClick }) => {
  return (
    <button
      className={clsx('button button--primary', 'urduTranslateButton')}
      onClick={onClick}
      title="Translate to Urdu"
      aria-label="Translate chapter to Urdu"
    >
      <span className="icon">
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="currentColor"
          className="svgBookIcon" // Apply CSS for the book icon
        >
          <path d="M21 4H3c-1.1 0-2 .9-2 2v13c0 1.1.9 2 2 2h18c1.1 0 2-.9 2-2V6c0-1.1-.9-2-2-2zM5 12h14v2H5v-2zm0-4h14v2H5V8zm0 8h14v2H5v-2z" />
        </svg>
      </span>
      <span className="text">اردو میں ترجمہ کریں</span>
    </button>
  );
};

export default UrduTranslateButton;
