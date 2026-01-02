import React, { useEffect } from 'react';
import { useAuth } from './AuthContext';
import { useHistory, useLocation } from '@docusaurus/router';
import styles from './styles.module.css';

interface ProtectedContentProps {
  children: React.ReactNode;
  requireAuth?: boolean;
  redirectTo?: string;
  showMessage?: boolean;
}

/**
 * ProtectedContent Component
 * Wraps content that requires authentication
 * Redirects to signup if user is not logged in
 */
export const ProtectedContent: React.FC<ProtectedContentProps> = ({
  children,
  requireAuth = true,
  redirectTo = '/',
  showMessage = true
}) => {
  const { isAuthenticated, isLoading } = useAuth();
  const location = useLocation();

  // Show loading state while checking auth
  if (isLoading) {
    return (
      <div className={styles.protectedLoading}>
        <div className={styles.spinner}></div>
        <p>Loading...</p>
      </div>
    );
  }

  // If authentication is not required, render children
  if (!requireAuth) {
    return <>{children}</>;
  }

  // If user is not authenticated, show signup message
  if (!isAuthenticated) {
    return (
      <div className={styles.protectedContentOverlay}>
        <div className={styles.protectedContentCard}>
          <div className={styles.lockIcon}>🔒</div>
          <h2>Sign Up to Access This Content</h2>
          <p>
            Create a free account to access our comprehensive humanoid robotics textbook
            with personalized content based on your background and learning goals.
          </p>

          <div className={styles.benefitsList}>
            <div className={styles.benefitItem}>
              <span className={styles.checkIcon}>✓</span>
              <span>Personalized learning path</span>
            </div>
            <div className={styles.benefitItem}>
              <span className={styles.checkIcon}>✓</span>
              <span>Code examples in your preferred language</span>
            </div>
            <div className={styles.benefitItem}>
              <span className={styles.checkIcon}>✓</span>
              <span>AI chatbot tutor</span>
            </div>
            <div className={styles.benefitItem}>
              <span className={styles.checkIcon}>✓</span>
              <span>Track your progress</span>
            </div>
          </div>

          <div className={styles.authActions}>
            <button
              className={styles.primaryButton}
              onClick={() => {
                // Trigger signup modal
                window.dispatchEvent(new CustomEvent('openSignupModal'));
              }}
            >
              Sign Up Free
            </button>
            <button
              className={styles.secondaryButton}
              onClick={() => {
                // Trigger signin modal
                window.dispatchEvent(new CustomEvent('openSigninModal'));
              }}
            >
              Already have an account? Sign In
            </button>
          </div>

          <p className={styles.privacyNote}>
            By signing up, you agree to our Terms of Service and Privacy Policy.
          </p>
        </div>
      </div>
    );
  }

  // User is authenticated, render children
  return <>{children}</>;
};

/**
 * Hook to check if current page requires authentication
 */
export const useRequiresAuth = (pathname: string): boolean => {
  // Pages that DON'T require auth (whitelist)
  const publicPages = [
    '/',
    '/blog',
    '/about',
  ];

  // Check if current page is public
  const isPublicPage = publicPages.some(page =>
    pathname === page || pathname.startsWith(`${page}/`)
  );

  // All docs pages require auth
  const requiresAuth = pathname.startsWith('/docs');

  return requiresAuth && !isPublicPage;
};
