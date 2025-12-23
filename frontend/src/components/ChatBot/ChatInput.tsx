import React, { useState, KeyboardEvent } from 'react';
import styles from './ChatInput.module.css';
import { JSX } from 'react';

interface Props {
  onSend: (message: string) => void;
  disabled?: boolean;
}

export default function ChatInput({ onSend, disabled }: Props): JSX.Element {
  const [input, setInput] = useState('');
  const handleSend = () => {
    if (input.trim() && !disabled) {
      onSend(input.trim());
      setInput('');
    }
  };
  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };
  return (
    <div className={styles.inputContainer}>
      <textarea
        className={styles.input}
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyDown={handleKeyDown}
        disabled={disabled}
        placeholder="Ask about humanoid robotics..."
        rows={1}
      />
      <button className={styles.sendButton} onClick={handleSend} disabled={disabled || !input.trim()}>
        {disabled ? '⏳' : '➤'}
      </button>
    </div>
  );
}