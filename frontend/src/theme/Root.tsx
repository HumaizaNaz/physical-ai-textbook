// frontend/src/theme/Root.tsx
import React, { ReactNode, useState, useEffect } from 'react';
import ChatBot from '@site/src/components/ChatBot/index';
import UrduTranslateButton from '@site/src/components/UrduTranslateButton'; // Removed .jsx
import UrduTranslationModal from '@site/src/components/UrduTranslationModal'; // Removed .jsx

interface RootProps {
  children?: ReactNode;
}

const Root: React.FC<RootProps> = ({ children }) => {
  const [showModal, setShowModal] = useState(false);
  const [translatedText, setTranslatedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [errorMessage, setErrorMessage] = useState('');

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
      <div style={{ position: 'fixed', bottom: '20px', left: '20px', zIndex: 1000 }}>
        <UrduTranslateButton onClick={translateCurrentChapter} />
      </div>

      <UrduTranslationModal
        isOpen={showModal}
        onClose={handleCloseModal}
        content={translatedText}
        isLoading={isLoading}
        errorMessage={errorMessage}
      />
      <ChatBot />
    </>
  );
};

export default Root;
