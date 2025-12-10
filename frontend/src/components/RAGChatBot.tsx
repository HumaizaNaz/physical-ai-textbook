import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './RAGChatBot.module.css';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

interface ChatMessage {
  text: string;
  isUser: boolean;
  sources?: string[];
}

const RAGChatBot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [input, setInput] = useState<string>('');
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [selectedTextContext, setSelectedTextContext] = useState<string | null>(null);
  const [selectedAgent, setSelectedAgent] = useState<string>("teaching-assistant");

  const CHAT_API_URL = 'http://localhost:8001/chat';

  const sendMessage = async (message: string, selectedContext: string | null = null) => {
    if (!message.trim() && !selectedContext) return;

    const userMessage: ChatMessage = { text: message, isUser: true };
    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const payload = {
        question: message,
        selected_text: selectedContext,
        agent_id: selectedAgent
      };

      const response = await fetch(CHAT_API_URL, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      setMessages((prev) => [
        ...prev,
        { text: data.answer, isUser: false, sources: data.sources },
      ]);
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages((prev) => [
        ...prev,
        { text: 'Oops! Something went wrong. Please try again.', isUser: false },
      ]);
    } finally {
      setIsLoading(false);
      setSelectedTextContext(null); // Clear context after sending
    }
  };

  const handleSendClick = () => {
    sendMessage(input, selectedTextContext);
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      handleSendClick();
    }
  };

  // --- Right-click "Ask AI about this" functionality ---
  useEffect(() => {
    const handleContextMenu = (e: MouseEvent) => {
      const selection = window.getSelection();
      if (selection && selection.toString().length > 0) {
        const selectedText = selection.toString();
        // Prevent default context menu
        e.preventDefault();

        // Create a custom menu item (simplified for quick implementation)
        const customMenuItem = document.createElement('div');
        customMenuItem.className = styles.customContextMenuItem;
        customMenuItem.textContent = 'Ask AI about this';
        customMenuItem.style.position = 'absolute';
        customMenuItem.style.left = `${e.clientX}px`;
        customMenuItem.style.top = `${e.clientY}px`;
        customMenuItem.style.background = 'var(--ifm-background-color)';
        customMenuItem.style.border = '1px solid var(--ifm-color-emphasis-300)';
        customMenuItem.style.padding = '8px';
        customMenuItem.style.borderRadius = '4px';
        customMenuItem.style.cursor = 'pointer';
        customMenuItem.style.zIndex = '99999';

        customMenuItem.onclick = () => {
          setIsOpen(true); // Open chatbot
          setSelectedTextContext(selectedText); // Set context
          setInput(`Regarding: "${selectedText}"`); // Pre-fill input with context
          document.body.removeChild(customMenuItem); // Remove menu
        };

        document.body.appendChild(customMenuItem);

        // Remove custom menu if user clicks elsewhere
        const handleClickOutside = () => {
          if (document.body.contains(customMenuItem)) {
            document.body.removeChild(customMenuItem);
          }
          document.removeEventListener('click', handleClickOutside);
        };
        document.addEventListener('click', handleClickOutside);
      }
    };

    document.addEventListener('contextmenu', handleContextMenu);

    return () => {
      document.removeEventListener('contextmenu', handleContextMenu);
    };
  }, []);

  return (
    <>
      <div
        className={styles.chatBubble}
        onClick={() => setIsOpen(!isOpen)}
        title="Open RAG Chatbot"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
        </svg>
      </div>

      <div className={clsx(styles.chatWindow, { [styles.chatWindowOpen]: isOpen })}>
        <div className={styles.chatHeader}>
          <span>Physical AI Textbook Chatbot</span>
          <button className={styles.closeButton} onClick={() => setIsOpen(false)}>
            &times;
          </button>
        </div>
        <div className={styles.chatMessages}>
          {messages.length === 0 && (
            <div className={styles.welcomeMessage}>
              Hi there! Ask me anything about the Physical AI Textbook.
            </div>
          )}
          {messages.map((msg, index) => (
            <div
              key={index}
              className={clsx(styles.chatMessage, { [styles.userMessage]: msg.isUser })}
            >
              {/* Use ReactMarkdown to render the text */}
              <ReactMarkdown remarkPlugins={[remarkGfm]}>{msg.text}</ReactMarkdown>
              {!msg.isUser && msg.sources && msg.sources.length > 0 && (
                <div className={styles.sources}>
                  <strong>Sources:</strong>
                  <ul>
                    {msg.sources.map((src, srcIndex) => (
                      <li key={srcIndex}>
                        <a href={src} target="_blank" rel="noopener noreferrer">
                          {new URL(src).pathname.split('/').pop() || src}
                        </a>
                      </li>
                    ))}
                  </ul>
                </div>
              )}
            </div>
          ))}
          {isLoading && <div className={styles.loadingIndicator}>Thinking...</div>}
        </div>
        {selectedTextContext && (
          <div className={styles.contextPreview}>
            Asking about: "<em>{selectedTextContext.substring(0, 100)}...</em>"
            <button onClick={() => setSelectedTextContext(null)} className={styles.clearContextButton}>
              Clear Context
            </button>
          </div>
        )}
        <div className={styles.chatInputContainer}>
          <input
            type="text"
            className={styles.chatInput}
            placeholder="Ask a question..."
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={handleKeyDown}
            disabled={isLoading}
          />
          <button className={styles.sendButton} onClick={handleSendClick} disabled={isLoading}>
            Send
          </button>
        </div>
      </div>
    </>
  );
};

export default RAGChatBot;