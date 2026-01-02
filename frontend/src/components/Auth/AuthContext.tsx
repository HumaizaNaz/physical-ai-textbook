// frontend/src/components/Auth/AuthContext.tsx
import React, { createContext, useState, useEffect, useContext, ReactNode } from 'react';

// Types
interface UserProfile {
  id: string;
  email: string;
  name?: string;
  programming_experience: string;
  ros_experience: string;
  linux_familiarity: string;
  hardware_experience: string;
  electronics_knowledge: string;
  robotics_projects: string;
  learning_goal: string;
  created_at: string;
  updated_at: string;
}

interface ProfileUpdateData {
  programming_experience?: string;
  ros_experience?: string;
  linux_familiarity?: string;
  hardware_experience?: string;
  electronics_knowledge?: string;
  robotics_projects?: string;
  learning_goal?: string;
}

interface AuthContextType {
  user: UserProfile | null;
  token: string | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  signin: (email: string, password: string) => Promise<void>;
  signup: (
    email: string,
    password: string,
    programming_experience: string,
    ros_experience?: string,
    linux_familiarity?: string,
    hardware_experience?: string,
    electronics_knowledge?: string,
    robotics_projects?: string,
    learning_goal?: string
  ) => Promise<void>;
  signout: () => void;
  updateProfile: (data: ProfileUpdateData) => Promise<void>;
  requestPasswordReset: (email: string) => Promise<void>;
  resetPassword: (token: string, newPassword: string) => Promise<void>;
  error: string | null;
  clearError: () => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

// API Base URL (points to backend-auth with Better-Auth)
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname !== 'localhost'
      ? 'https://better-auth-taupe-nine.vercel.app' // Deployed backend-auth URL
      : process.env.NODE_ENV === 'development'
        ? 'http://localhost:7860' // Local development
        : 'https://better-auth-taupe-nine.vercel.app'; // Production but accessed from localhost
// API Functions
async function signinAPI(email: string, password: string) {
  const response = await fetch(`${API_BASE_URL}/api/auth/sign-in/email`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, password })
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error || 'Signin failed');
  }

  const result = await response.json();

  // Store token in localStorage
  if (result.access_token) {
    localStorage.setItem('auth_token', result.access_token);
  }

  return result;
}

async function signupAPI(
  email: string,
  password: string,
  programming_experience: string,
  ros_experience?: string,
  linux_familiarity?: string,
  hardware_experience?: string,
  electronics_knowledge?: string,
  robotics_projects?: string,
  learning_goal?: string
) {
  const response = await fetch(`${API_BASE_URL}/api/auth/sign-up/email`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      email,
      password,
      name: email.split('@')[0] // Extract name from email
    })
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error || 'Signup failed');
  }

  const result = await response.json();

  // Store token in localStorage
  if (result.access_token) {
    localStorage.setItem('auth_token', result.access_token);
  }

  // Update user profile with additional fields
  if (result.user && result.access_token) {
    await fetch(`${API_BASE_URL}/api/user/background`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${result.access_token}`
      },
      body: JSON.stringify({
        programming_experience,
        ros_experience,
        linux_familiarity,
        hardware_experience,
        electronics_knowledge,
        robotics_projects,
        learning_goal
      })
    });
  }

  return result;
}

async function requestPasswordResetAPI(email: string) {
  const response = await fetch(`${API_BASE_URL}/api/auth/reset-password/request`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email })
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error || 'Password reset request failed');
  }

  return response.json();
}

async function resetPasswordAPI(token: string, newPassword: string) {
  const response = await fetch(`${API_BASE_URL}/api/auth/reset-password`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ token, newPassword })
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error || 'Password reset failed');
  }

  return response.json();
}

async function updateProfileAPI(token: string, data: ProfileUpdateData) {
  const response = await fetch(`${API_BASE_URL}/api/user/background`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${token}`
    },
    body: JSON.stringify(data)
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error || 'Profile update failed');
  }

  return response.json();
}

// Provider Component
export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<UserProfile | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Check for existing token on mount
  useEffect(() => {
    const storedToken = localStorage.getItem('auth_token');
    if (storedToken) {
      // Verify token and get user profile
      fetch(`${API_BASE_URL}/api/user/me`, {
        headers: { 'Authorization': `Bearer ${storedToken}` }
      })
      .then(response => {
        if (response.ok) {
          return response.json();
        }
        // If token is invalid, remove it
        localStorage.removeItem('auth_token');
        throw new Error('Token invalid');
      })
      .then(profile => {
        setUser({
          id: profile.id,
          email: profile.email,
          name: profile.name,
          programming_experience: profile.programming_experience || 'None',
          ros_experience: profile.ros_experience || 'None',
          linux_familiarity: profile.linux_familiarity || 'Beginner',
          hardware_experience: profile.hardware_experience || 'None',
          electronics_knowledge: profile.electronics_knowledge || 'None',
          robotics_projects: profile.robotics_projects || 'None',
          learning_goal: profile.learning_goal || 'General Learning',
          created_at: profile.created_at || new Date().toISOString(),
          updated_at: profile.updated_at || new Date().toISOString(),
        });
        setToken(storedToken);
      })
      .catch(() => {
        // Token invalid, clear it
        localStorage.removeItem('auth_token');
      })
      .finally(() => {
        setIsLoading(false);
      });
    } else {
      setIsLoading(false);
    }
  }, []);

  const signin = async (email: string, password: string) => {
    try {
      setError(null);
      const response = await signinAPI(email, password);
      const userData = response.user;
      const sessionToken = response.session?.token || response.access_token;
      
      // Store token in localStorage
      localStorage.setItem('auth_token', sessionToken);
      setToken(sessionToken);
      
      // Set user data
      setUser({
        id: userData.id,
        email: userData.email,
        name: userData.name,
        programming_experience: userData.programming_experience || 'None',
        ros_experience: userData.ros_experience || 'None',
        linux_familiarity: userData.linux_familiarity || 'Beginner',
        hardware_experience: userData.hardware_experience || 'None',
        electronics_knowledge: userData.electronics_knowledge || 'None',
        robotics_projects: userData.robotics_projects || 'None',
        learning_goal: userData.learning_goal || 'General Learning',
        created_at: userData.created_at || new Date().toISOString(),
        updated_at: userData.updated_at || new Date().toISOString(),
      });
    } catch (err: any) {
      setError(err.message);
      throw err;
    }
  };

  const signup = async (
    email: string,
    password: string,
    programming_experience: string,
    ros_experience: string = 'None',
    linux_familiarity: string = 'Beginner',
    hardware_experience: string = 'None',
    electronics_knowledge: string = 'None',
    robotics_projects: string = 'None',
    learning_goal: string = 'General Learning'
  ) => {
    try {
      setError(null);
      const response = await signupAPI(
        email,
        password,
        programming_experience,
        ros_experience,
        linux_familiarity,
        hardware_experience,
        electronics_knowledge,
        robotics_projects,
        learning_goal
      );
      const userData = response.user;
      const sessionToken = response.session?.token || response.access_token;
      
      // Store token in localStorage
      localStorage.setItem('auth_token', sessionToken);
      setToken(sessionToken);
      
      // Set user data
      setUser({
        id: userData.id,
        email: userData.email,
        name: userData.name,
        programming_experience: programming_experience,
        ros_experience: ros_experience,
        linux_familiarity: linux_familiarity,
        hardware_experience: hardware_experience,
        electronics_knowledge: electronics_knowledge,
        robotics_projects: robotics_projects,
        learning_goal: learning_goal,
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
      });
    } catch (err: any) {
      setError(err.message);
      throw err;
    }
  };

  const signout = () => {
    localStorage.removeItem('auth_token');
    setToken(null);
    setUser(null);
    setError(null);
  };

  const updateProfile = async (data: ProfileUpdateData) => {
    if (!token) throw new Error('Not authenticated');

    try {
      setError(null);
      const updatedUser = await updateProfileAPI(token, data);
      setUser(prevUser => ({
        ...prevUser!,
        ...data,
        updated_at: new Date().toISOString(),
      }));
    } catch (err: any) {
      setError(err.message);
      throw err;
    }
  };

  const requestPasswordReset = async (email: string) => {
    try {
      setError(null);
      await requestPasswordResetAPI(email);
      // Success message can be handled by the calling component
    } catch (err: any) {
      setError(err.message);
      throw err;
    }
  };

  const resetPassword = async (token: string, newPassword: string) => {
    try {
      setError(null);
      await resetPasswordAPI(token, newPassword);
      // Success message can be handled by the calling component
    } catch (err: any) {
      setError(err.message);
      throw err;
    }
  };

  const clearError = () => setError(null);

  return (
    <AuthContext.Provider value={{
      user,
      token,
      isAuthenticated: !!user,
      isLoading,
      signin,
      signup,
      signout,
      updateProfile,
      requestPasswordReset,
      resetPassword,
      error,
      clearError,
    }}>
      {children}
    </AuthContext.Provider>
  );
};

// Custom Hook
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};