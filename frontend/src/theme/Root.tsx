// frontend/src/theme/Root.tsx
import React, { ReactNode } from 'react';
// Update import path to .tsx
import ChatBot from '../components/ChatBot/index';

interface RootProps {
  children?: ReactNode;
}

const Root: React.FC<RootProps> = ({ children }) => {
  return (
    <>
      {children}
      <ChatBot />
    </>
  );
};

export default Root;