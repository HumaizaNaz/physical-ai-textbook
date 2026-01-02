// frontend/src/theme/Root.tsx
import React, { ReactNode, useState, useEffect } from 'react';
import ChatBot from '@site/src/components/ChatBot/index';
import UrduTranslateButton from '@site/src/components/UrduTranslateButton';
import UrduTranslationModal from '@site/src/components/UrduTranslationModal';
import { PasswordResetModal } from '@site/src/components/Auth/PasswordResetModal';

import { AuthProvider } from '@site/src/components/Auth';
import { UIProvider } from '@site/src/components/UI/UIContext';


interface RootProps {
  children?: ReactNode;
}

const RootComponent: React.FC<RootProps> = ({ children }) => {
  const [showModal, setShowModal] = useState(false);
  const [translatedText, setTranslatedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [errorMessage, setErrorMessage] = useState('');
  const [isChapterPage, setIsChapterPage] = useState(false);
  const [showPasswordResetModal, setShowPasswordResetModal] = useState(false);

  useEffect(() => {
    const handlePathChange = () => {
      if (window.location.pathname.includes('/docs/')) {
        setIsChapterPage(true);
      } else {
        setIsChapterPage(false);
      }
    };

    handlePathChange(); // Initial check

    // Docusaurus uses client-side routing, so we need to listen for changes
    const observer = new MutationObserver(handlePathChange);
    observer.observe(document.body, { childList: true, subtree: true });

    // Handle custom events for opening modals
    const handleOpenPasswordResetModal = () => {
      setShowPasswordResetModal(true);
    };

    window.addEventListener('openPasswordResetModal', handleOpenPasswordResetModal);

    return () => {
      observer.disconnect();
      window.removeEventListener('openPasswordResetModal', handleOpenPasswordResetModal);
    };
  }, []); 


  const extractChapterText = (): string => {
    const mainContent =
      document.querySelector('main') || document.querySelector('.docMainContainer');
    if (mainContent) {
      return mainContent.innerText;
    }
    return '';
  };

  const translateCurrentChapter = async () => {
    setErrorMessage('');
    setIsLoading(true);
    setTranslatedText('');
    setShowModal(true);
    const chapterText = extractChapterText();

    if (!chapterText) {
      setErrorMessage('Could not extract chapter text for translation. Please ensure you are on a chapter page with content.');
      setIsLoading(false);
      return;
    }

    try {
      const response = await fetch('https://humaiza-rag-chatbot.hf.space/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question:
            'Translate this entire chapter to natural Roman Urdu for Pakistani students. Keep technical terms in English and code blocks unchanged.',
          selected_text: chapterText,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      setTranslatedText(data.answer);
    } catch (error) {
      console.error('Translation failed:', error);
      setErrorMessage(`Translation failed: ${error.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  const handleCloseModal = () => {
    setShowModal(false);
    setTranslatedText('');
    setErrorMessage('');
  };

  return (
    <>
      {children}
      <div style={{ position: 'fixed', bottom: '20px', left: '20px', zIndex: 1000, display: 'flex', gap: '10px' }}>
        <UrduTranslateButton onClick={translateCurrentChapter} />
        {isChapterPage }
      </div>

      <UrduTranslationModal
        isOpen={showModal}
        onClose={handleCloseModal}
        content={translatedText}
        isLoading={isLoading}
        errorMessage={errorMessage}
      />
      <PasswordResetModal
        isOpen={showPasswordResetModal}
        onClose={() => setShowPasswordResetModal(false)}
        onSwitchToSignin={() => {
          setShowPasswordResetModal(false);
          // Trigger the signin modal
          window.dispatchEvent(new CustomEvent('openSigninModal'));
        }}
      />
      <ChatBot />
    </>
  );
};

const Root: React.FC<RootProps> = ({ children }) => {
  return (
    <AuthProvider>
      <UIProvider>
        <RootComponent>{children}</RootComponent>
      </UIProvider>
    </AuthProvider>
  );
};


export default Root;