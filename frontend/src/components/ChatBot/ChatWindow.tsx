import React, { useEffect, useRef } from 'react';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import styles from './ChatWindow.module.css';
import { JSX } from 'react';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: string[];
}

interface Props {
  onClose: () => void;
  messages: Message[];
  onSendMessage: (message: string) => void;
  loading: boolean;
}

export default function ChatWindow({ onClose, messages, onSendMessage, loading }: Props): JSX.Element {
  const messagesEndRef = useRef(null);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  return (
    <div className={styles.window} onClick={e => e.stopPropagation()}>
        <div className={styles.header}>
          <div className={styles.headerLeft}>
            <h3 className={styles.title}>Physical AI Assistant</h3>
            <span className={styles.badge}>👩‍💻</span>
          </div>
          <button className={styles.closeButton} onClick={onClose}>✕</button>
        </div>
        <div className={styles.messages}>
          {messages.map((msg, idx) => <ChatMessage key={idx} {...msg} />)}
          {loading && <ChatMessage role="assistant" content="Thinking..." isStreaming={true} />}
          <div ref={messagesEndRef} />
        </div>
        <ChatInput onSend={onSendMessage} disabled={loading} />
        <div className={styles.footer}>
          <p className={styles.footerText}>Powered by Cohere RAG</p>
        </div>
      </div>

  );
}