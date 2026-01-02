// frontend/src/clientModules/navbarAuthInjector.ts
import { useUI } from '@site/src/components/UI/UIContext';

// This module registers event listeners to open modals from anywhere in the app
export default function navbarAuthInjector() {
  // Listen for custom events to open modals
  const handleOpenSignupModal = () => {
    // We'll dispatch an event that the Root component can handle
    window.dispatchEvent(new CustomEvent('openSignupModal'));
  };

  const handleOpenSigninModal = () => {
    window.dispatchEvent(new CustomEvent('openSigninModal'));
  };

  const handleOpenPasswordResetModal = () => {
    window.dispatchEvent(new CustomEvent('openPasswordResetModal'));
  };

  // Add event listeners when the module loads
  window.addEventListener('openSignupModal', handleOpenSignupModal);
  window.addEventListener('openSigninModal', handleOpenSigninModal);
  window.addEventListener('openPasswordResetModal', handleOpenPasswordResetModal);

  // Clean up event listeners when the module unloads
  return () => {
    window.removeEventListener('openSignupModal', handleOpenSignupModal);
    window.removeEventListener('openSigninModal', handleOpenSigninModal);
    window.removeEventListener('openPasswordResetModal', handleOpenPasswordResetModal);
  };
}