// frontend/src/components/UI/UIContext.tsx
import React, { createContext, useContext, useState, ReactNode } from 'react';

interface UIContextType {
  isSignupModalOpen: boolean;
  isSigninModalOpen: boolean;
  isProfileSettingsModalOpen: boolean;
  isPasswordResetModalOpen: boolean;
  openSignupModal: () => void;
  closeSignupModal: () => void;
  openSigninModal: () => void;
  closeSigninModal: () => void;
  openProfileSettingsModal: () => void;
  closeProfileSettingsModal: () => void;
  openPasswordResetModal: () => void;
  closePasswordResetModal: () => void;
  // Function to switch between sign-in and sign-up
  switchToSignup: () => void;
  switchToSignin: () => void;
  switchToPasswordReset: () => void;
}

const UIContext = createContext<UIContextType | undefined>(undefined);

export const UIProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [isSignupModalOpen, setIsSignupModalOpen] = useState(false);
  const [isSigninModalOpen, setIsSigninModalOpen] = useState(false);
  const [isProfileSettingsModalOpen, setIsProfileSettingsModalOpen] = useState(false);
  const [isPasswordResetModalOpen, setIsPasswordResetModalOpen] = useState(false);

  const openSignupModal = () => {
    setIsSignupModalOpen(true);
    setIsSigninModalOpen(false); // Close signin if open
    setIsProfileSettingsModalOpen(false); // Close profile if open
  };
  const closeSignupModal = () => setIsSignupModalOpen(false);

  const openSigninModal = () => {
    setIsSigninModalOpen(true);
    setIsSignupModalOpen(false); // Close signup if open
    setIsProfileSettingsModalOpen(false); // Close profile if open
  };
  const closeSigninModal = () => setIsSigninModalOpen(false);

  const openProfileSettingsModal = () => {
    setIsProfileSettingsModalOpen(true);
    setIsSignupModalOpen(false); // Close signup if open
    setIsSigninModalOpen(false); // Close signin if open
  };
  const closeProfileSettingsModal = () => setIsProfileSettingsModalOpen(false);

  const openPasswordResetModal = () => {
    setIsPasswordResetModalOpen(true);
    setIsSignupModalOpen(false); // Close signup if open
    setIsSigninModalOpen(false); // Close signin if open
    setIsProfileSettingsModalOpen(false); // Close profile if open
  };
  const closePasswordResetModal = () => setIsPasswordResetModalOpen(false);

  const switchToSignup = () => {
    openSignupModal();
  };

  const switchToSignin = () => {
    openSigninModal();
  };

  const switchToPasswordReset = () => {
    openPasswordResetModal();
  };

  return (
    <UIContext.Provider
      value={{
        isSignupModalOpen,
        isSigninModalOpen,
        isProfileSettingsModalOpen,
        isPasswordResetModalOpen,
        openSignupModal,
        closeSignupModal,
        openSigninModal,
        closeSigninModal,
        openProfileSettingsModal,
        closeProfileSettingsModal,
        openPasswordResetModal,
        closePasswordResetModal,
        switchToSignup,
        switchToSignin,
        switchToPasswordReset,
      }}
    >
      {children}
    </UIContext.Provider>
  );
};

export const useUI = () => {
  const context = useContext(UIContext);
  if (context === undefined) {
    throw new Error('useUI must be used within a UIProvider');
  }
  return context;
};
