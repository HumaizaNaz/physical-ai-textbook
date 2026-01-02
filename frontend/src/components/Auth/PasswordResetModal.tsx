// frontend/src/components/Auth/PasswordResetModal.tsx
import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import styles from './styles.module.css';

interface PasswordResetModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSwitchToSignin: () => void;
}

export const PasswordResetModal: React.FC<PasswordResetModalProps> = ({ 
  isOpen, 
  onClose, 
  onSwitchToSignin 
}) => {
  const [step, setStep] = useState<'request' | 'reset'>('request');
  const [email, setEmail] = useState('');
  const [token, setToken] = useState('');
  const [newPassword, setNewPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');
  
  const { requestPasswordReset, resetPassword } = useAuth();

  const handleRequestReset = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setMessage('');
    
    if (!email) {
      setError('Email is required');
      return;
    }

    try {
      setIsSubmitting(true);
      await requestPasswordReset(email);
      setMessage('Password reset instructions have been sent to your email.');
      setStep('reset');
    } catch (err: any) {
      setError(err.message || 'Failed to send password reset instructions');
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleResetPassword = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    
    if (newPassword !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (newPassword.length < 8) {
      setError('Password must be at least 8 characters long');
      return;
    }

    try {
      setIsSubmitting(true);
      await resetPassword(token, newPassword);
      setMessage('Password reset successfully. You can now sign in with your new password.');
      setTimeout(() => {
        setStep('request');
        setToken('');
        setNewPassword('');
        setConfirmPassword('');
        onSwitchToSignin();
      }, 2000);
    } catch (err: any) {
      setError(err.message || 'Failed to reset password');
    } finally {
      setIsSubmitting(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className={styles.modalOverlay} onClick={onClose}>
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        <button className={styles.modalClose} onClick={onClose}>
          &times;
        </button>
        
        <div className={styles.modalBody}>
          {step === 'request' ? (
            <>
              <h2 className={styles.modalTitle}>Reset Password</h2>
              <p className={styles.modalSubtitle}>
                Enter your email address and we'll send you a link to reset your password.
              </p>
              
              {error && <div className={styles.errorMessage}>{error}</div>}
              {message && <div className={styles.successMessage}>{message}</div>}
              
              <form onSubmit={handleRequestReset} className={styles.authForm}>
                <div className={styles.formGroup}>
                  <label htmlFor="reset-email">Email</label>
                  <input
                    type="email"
                    id="reset-email"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    placeholder="your@email.com"
                    required
                  />
                </div>
                
                <button 
                  type="submit" 
                  className={styles.authButton}
                  disabled={isSubmitting}
                >
                  {isSubmitting ? 'Sending...' : 'Send Reset Link'}
                </button>
              </form>
              
              <p className={styles.switchAuth}>
                Remember your password?{' '}
                <button 
                  type="button" 
                  onClick={onSwitchToSignin}
                  className={styles.linkButton}
                >
                  Sign in
                </button>
              </p>
            </>
          ) : (
            <>
              <h2 className={styles.modalTitle}>Reset Your Password</h2>
              <p className={styles.modalSubtitle}>
                Enter the reset token sent to your email and your new password.
              </p>
              
              {error && <div className={styles.errorMessage}>{error}</div>}
              {message && <div className={styles.successMessage}>{message}</div>}
              
              <form onSubmit={handleResetPassword} className={styles.authForm}>
                <div className={styles.formGroup}>
                  <label htmlFor="reset-token">Reset Token</label>
                  <input
                    type="text"
                    id="reset-token"
                    value={token}
                    onChange={(e) => setToken(e.target.value)}
                    placeholder="Enter reset token"
                    required
                  />
                </div>
                
                <div className={styles.formGroup}>
                  <label htmlFor="new-password">New Password</label>
                  <input
                    type="password"
                    id="new-password"
                    value={newPassword}
                    onChange={(e) => setNewPassword(e.target.value)}
                    placeholder="Enter new password"
                    required
                  />
                </div>
                
                <div className={styles.formGroup}>
                  <label htmlFor="confirm-password">Confirm New Password</label>
                  <input
                    type="password"
                    id="confirm-password"
                    value={confirmPassword}
                    onChange={(e) => setConfirmPassword(e.target.value)}
                    placeholder="Confirm new password"
                    required
                  />
                </div>
                
                <button 
                  type="submit" 
                  className={styles.authButton}
                  disabled={isSubmitting}
                >
                  {isSubmitting ? 'Resetting...' : 'Reset Password'}
                </button>
              </form>
              
              <p className={styles.switchAuth}>
                <button 
                  type="button" 
                  onClick={() => setStep('request')}
                  className={styles.linkButton}
                >
                  Back to email request
                </button>
              </p>
            </>
          )}
        </div>
      </div>
    </div>
  );
};