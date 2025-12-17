import React, { useState } from 'react';
import FloatingChatButton from './FloatingChatButton';
import ChatWindow from './ChatWindow';
import styles from './ChatBot.module.css';
import { sendChatMessage } from './api'; // Import the API function
import { JSX } from 'react';

export default function ChatBot(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [loading, setLoading] = useState(false); // New loading state for the parent

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async (message) => {
    setMessages(prev => [...prev, { role: 'user', content: message }]);
    setLoading(true); // Set loading true when sending message
    try {
      // Assuming sendChatMessage is capable of handling the question and returning structured data
      const data = await sendChatMessage({ question: message, selected_text: '' });
      setMessages(prev => [...prev, { role: 'assistant', content: data.answer, sources: data.sources }]);
    } catch (e) {
      setMessages(prev => [...prev, { role: 'assistant', content: 'Sorry, error: ' + e.message }]);
    }
    setLoading(false); // Set loading false after response
  };

  return (
    <div className={styles.chatBotContainer}>
      <FloatingChatButton onClick={toggleChat} isOpen={isOpen} />
      {isOpen && (
        <ChatWindow
          onClose={toggleChat}
          messages={messages}
          onSendMessage={handleSendMessage}
          loading={loading} // Pass loading state to ChatWindow
        />
      )}
    </div>
  );
}
