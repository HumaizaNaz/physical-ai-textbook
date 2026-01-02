import React, { useState } from 'react';
import { useAuth } from '../Auth/AuthContext';
import { SignupModal } from '../Auth/SignupModal';
import { SigninModal } from '../Auth/SigninModal';
import { ProfileSettingsModal } from '../Auth/ProfileSettingsModal';
import { PasswordResetModal } from '../Auth/PasswordResetModal';
import styles from './styles.module.css';
import { JSX } from 'react';
import { useUI } from '../UI/UIContext';

export default function NavbarAuthButton(): JSX.Element {
  const { user, isAuthenticated, signout } = useAuth();
  const {
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
  } = useUI();
  const [showDropdown, setShowDropdown] = useState(false);

  if (!isAuthenticated) {
    return (
      <>
        <div className={styles.authButtons}>
          <button
            className={styles.signinButton}
            onClick={openSigninModal}
            aria-label="Sign In"
          >
            Sign In
          </button>
          <button
            className={styles.signupButton}
            onClick={openSignupModal}
            aria-label="Sign Up"
          >
            Sign Up
          </button>
        </div>

        {isSigninModalOpen && (
          <SigninModal
            onClose={closeSigninModal}
            onSwitchToSignup={switchToSignup}
            onSwitchToPasswordReset={switchToPasswordReset}
          />
        )}

        {isPasswordResetModalOpen && (
          <PasswordResetModal
            isOpen={isPasswordResetModalOpen}
            onClose={closePasswordResetModal}
            onSwitchToSignin={switchToSignin}
          />
        )}

        {isSignupModalOpen && (
          <SignupModal
            onClose={closeSignupModal}
            onSwitchToSignin={switchToSignin}
          />
        )}
      </>
    );
  }

  return (
    <>
      <div className={styles.profileContainer}>
        <button
          className={styles.profileButton}
          onClick={() => setShowDropdown(!showDropdown)}
          aria-label="User Profile"
          aria-expanded={showDropdown}
        >
          <span className={styles.profileIcon}>👨🏻‍💼</span>
        </button>

        {showDropdown && (
          <div className={styles.dropdown}>
            <div className={styles.dropdownHeader}>
              <span className={styles.dropdownUserIcon}>👨🏻‍💼</span>
              <div className={styles.dropdownUserInfo}>
                <div className={styles.dropdownUserName}>{user?.email?.split('@')[0]}</div>
                <div className={styles.dropdownUserEmail} title={user?.email}>{user?.email?.substring(0, 20) + (user?.email?.length > 20 ? '...' : '')}</div>
              </div>
            </div>

            <div className={styles.dropdownMenu}>
              <button
                className={styles.dropdownItem}
                onClick={() => {
                  openProfileSettingsModal();
                  setShowDropdown(false);
                }}
              >
                <span className={styles.dropdownIcon}>⚙️</span>
                <span>Profile Settings</span>
              </button>
              <button
                className={styles.dropdownItem}
                onClick={() => {
                  signout();
                  setShowDropdown(false);
                }}
              >
                <span className={styles.dropdownIcon}>🚪</span>
                <span>Sign Out</span>
              </button>
            </div>
          </div>
        )}
      </div>


      {isProfileSettingsModalOpen && (
        <ProfileSettingsModal isOpen={isProfileSettingsModalOpen} onClose={closeProfileSettingsModal} />
      )}
    </>
  );
}
