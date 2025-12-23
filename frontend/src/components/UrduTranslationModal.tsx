import React, { useEffect } from 'react';
import clsx from 'clsx';
import Markdown from 'react-markdown';
import '../css/UrduTranslate.css'; // Import for side effects

interface UrduTranslationModalProps {
  isOpen: boolean;
  onClose: () => void;
  content: string;
  isLoading: boolean;
  errorMessage: string;
}

const UrduTranslationModal: React.FC<UrduTranslationModalProps> = ({ isOpen, onClose, content, isLoading, errorMessage }) => {
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = 'unset';
    }
    return () => {
      document.body.style.overflow = 'unset';
    };
  }, [isOpen]);

  if (!isOpen) return null;

  return (
    <div className={clsx('modalOverlay')} onClick={onClose}>
      <div className={clsx('modalContent')} onClick={(e) => e.stopPropagation()}>
        <button className={clsx('modalCloseButton')} onClick={onClose} aria-label="Close translation modal">
          &times;
        </button>
        <h3 className={clsx('modalTitle')}>📕 Urdu Translation</h3>
        <div className={clsx('modalBody')}>
          {isLoading && <p className={clsx('modalLoadingText')}>ترجمہ ہو رہا ہے... (Translating...)</p>}
          {errorMessage && <p className={clsx('modalErrorText')}>Error: {errorMessage}</p>}
          {!isLoading && !errorMessage && <Markdown>{content}</Markdown>}
        </div>
      </div>
    </div>
  );
};

export default UrduTranslationModal;