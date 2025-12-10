import React from 'react';
import type { PropsWithChildren } from 'react';
import RAGChatBot from '@site/src/components/RAGChatBot';
import UrduTranslateButton from '@site/src/components/UrduTranslateButton'; // Import the new component
import { JSX } from 'react';

// Default implementation, that you might want to customize
export default function Root({ children }: PropsWithChildren): JSX.Element {
  return (
    <>
      {children}
      <RAGChatBot />

    </>
  );
}