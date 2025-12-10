import React from 'react';
import type { PropsWithChildren } from 'react';
import RAGChatBot from '@site/src/components/RAGChatBot';

// Default implementation, that you might want to customize
export default function Root({ children }: PropsWithChildren): JSX.Element {
  return (
    <>
      {children}
      <RAGChatBot />
    </>
  );
}