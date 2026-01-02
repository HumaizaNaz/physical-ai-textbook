import React, { useState } from 'react';
import { createPortal } from 'react-dom';
import { useHistory } from '@docusaurus/router';
import { useAuth } from './AuthContext';
import styles from './styles.module.css';

interface SignupModalProps {
  onClose: () => void;
  onSwitchToSignin: () => void;
  onSwitchToPasswordReset: () => void;
}

export const SignupModal: React.FC<SignupModalProps> = ({ onClose, onSwitchToSignin, onSwitchToPasswordReset }) => {
  const { signup } = useAuth();
  const history = useHistory();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  // Software Background
  const [programmingExperience, setProgrammingExperience] = useState('None');
  const [rosExperience, setRosExperience] = useState('None');
  const [linuxFamiliarity, setLinuxFamiliarity] = useState('Beginner');

  // Hardware Background
  const [hardwareExperience, setHardwareExperience] = useState('None');
  const [electronicsKnowledge, setElectronicsKnowledge] = useState('None');
  const [roboticsProjects, setRoboticsProjects] = useState('None');

  // Learning Goals
  const [learningGoal, setLearningGoal] = useState('General Learning');

  const [showPassword, setShowPassword] = useState(false);
  const [error, setError] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);

  const getPasswordStrength = (pwd: string): string => {
    if (pwd.length === 0) return '';
    if (pwd.length < 8) return 'Weak';

    let strength = 0;
    if (/[a-z]/.test(pwd)) strength++;
    if (/[A-Z]/.test(pwd)) strength++;
    if (/[0-9]/.test(pwd)) strength++;
    if (/[^a-zA-Z0-9]/.test(pwd)) strength++;

    if (strength <= 2) return 'Weak';
    if (strength === 3) return 'Medium';
    return 'Strong';
  };

  const passwordStrength = getPasswordStrength(password);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setIsSubmitting(true);

    try {
      await signup(
        email,
        password,
        programmingExperience,
        rosExperience,
        linuxFamiliarity,
        hardwareExperience,
        electronicsKnowledge,
        roboticsProjects,
        learningGoal
      );
      // Reset form
      setEmail('');
      setPassword('');
      setProgrammingExperience('None');
      setRosExperience('None');
      setLinuxFamiliarity('Beginner');
      setHardwareExperience('None');
      setElectronicsKnowledge('None');
      setRoboticsProjects('None');
      setLearningGoal('General Learning');
      onClose();
      // Redirect to homepage after successful signup
      history.push('/');
    } catch (err: any) {
      if (err.message.includes('already registered')) {
        setError('Email already registered. Please sign in instead.');
      } else if (err.message.includes('Password')) {
        setError('Password must be at least 8 characters with uppercase, lowercase, and number.');
      } else {
        setError(err.message || 'Signup failed. Please try again.');
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleClose = () => {
    setEmail('');
    setPassword('');
    setProgrammingExperience('None');
    setRosExperience('None');
    setLinuxFamiliarity('Beginner');
    setHardwareExperience('None');
    setElectronicsKnowledge('None');
    setRoboticsProjects('None');
    setLearningGoal('General Learning');
    setError('');
    onClose();
  };

  // Render modal at document body level using Portal
  if (typeof document === 'undefined') return null;

  return createPortal(
    <div className={styles.modalOverlay} onClick={handleClose}>
      <div className={`${styles.modalContent} ${styles.scrollableModal}`} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={handleClose} aria-label="Close">
          ×
        </button>

        <h2 className={styles.modalTitle}>Sign Up</h2>
        <p className={styles.modalSubtitle}>Tell us about your background for personalized content</p>

        {error && <div className={styles.errorMessage}>{error}</div>}

        <form onSubmit={handleSubmit} className={styles.authForm}>
          {/* Authentication */}
          <div className={styles.formGroup}>
            <label htmlFor="signup-email">Email</label>
            <input
              id="signup-email"
              type="email"
              placeholder="your.email@example.com"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
              autoComplete="email"
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="signup-password">Password</label>
            <div className={styles.passwordField}>
              <input
                id="signup-password"
                type={showPassword ? 'text' : 'password'}
                placeholder="Min 8 characters"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                minLength={8}
                autoComplete="new-password"
              />
              <button
                type="button"
                className={styles.passwordToggle}
                onClick={() => setShowPassword(!showPassword)}
                aria-label={showPassword ? 'Hide password' : 'Show password'}
              >
                {showPassword ? '👁️' : '👁️‍🗨️'}
              </button>
            </div>
            {password && (
              <div className={`${styles.passwordStrength} ${styles[passwordStrength.toLowerCase()]}`}>
                {passwordStrength}
              </div>
            )}
          </div>

          {/* Software Background Section */}
          <div className={styles.sectionDivider}>
            <span className={styles.sectionTitle}>Software Background</span>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="signup-programming">Programming Experience *</label>
            <select
              id="signup-programming"
              value={programmingExperience}
              onChange={(e) => setProgrammingExperience(e.target.value)}
              required
            >
              <option value="None">None (I'm a beginner)</option>
              <option value="Python">Python</option>
              <option value="C++">C++</option>
              <option value="Both Python and C++">Both Python and C++</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="signup-ros">ROS Experience</label>
            <select
              id="signup-ros"
              value={rosExperience}
              onChange={(e) => setRosExperience(e.target.value)}
            >
              <option value="None">None</option>
              <option value="ROS1">ROS1</option>
              <option value="ROS2">ROS2</option>
              <option value="Both ROS1 and ROS2">Both ROS1 and ROS2</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="signup-linux">Linux Familiarity</label>
            <select
              id="signup-linux"
              value={linuxFamiliarity}
              onChange={(e) => setLinuxFamiliarity(e.target.value)}
            >
              <option value="Beginner">Beginner</option>
              <option value="Intermediate">Intermediate</option>
              <option value="Advanced">Advanced</option>
            </select>
          </div>

          {/* Hardware Background Section */}
          <div className={styles.sectionDivider}>
            <span className={styles.sectionTitle}>Hardware Background</span>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="signup-hardware">Hardware Experience</label>
            <select
              id="signup-hardware"
              value={hardwareExperience}
              onChange={(e) => setHardwareExperience(e.target.value)}
            >
              <option value="None">None</option>
              <option value="Arduino">Arduino</option>
              <option value="Raspberry Pi">Raspberry Pi</option>
              <option value="Both Arduino and Raspberry Pi">Both Arduino and Raspberry Pi</option>
              <option value="Advanced (Custom Boards)">Advanced (Custom Boards)</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="signup-electronics">Electronics Knowledge</label>
            <select
              id="signup-electronics"
              value={electronicsKnowledge}
              onChange={(e) => setElectronicsKnowledge(e.target.value)}
            >
              <option value="None">None</option>
              <option value="Basic (Can read schematics)">Basic (Can read schematics)</option>
              <option value="Intermediate (Can design circuits)">Intermediate (Can design circuits)</option>
              <option value="Advanced (PCB design)">Advanced (PCB design)</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="signup-projects">Robotics Projects Completed</label>
            <select
              id="signup-projects"
              value={roboticsProjects}
              onChange={(e) => setRoboticsProjects(e.target.value)}
            >
              <option value="None">None</option>
              <option value="1-2 projects">1-2 projects</option>
              <option value="3-5 projects">3-5 projects</option>
              <option value="5+ projects">5+ projects</option>
            </select>
          </div>

          {/* Learning Goals Section */}
          <div className={styles.sectionDivider}>
            <span className={styles.sectionTitle}>Learning Goals</span>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="signup-goal">Primary Learning Goal</label>
            <select
              id="signup-goal"
              value={learningGoal}
              onChange={(e) => setLearningGoal(e.target.value)}
            >
              <option value="General Learning">General Learning</option>
              <option value="Robot Manipulation">Robot Manipulation</option>
              <option value="Humanoid Locomotion">Humanoid Locomotion</option>
              <option value="Computer Vision">Computer Vision</option>
              <option value="Control Systems">Control Systems</option>
              <option value="ROS Development">ROS Development</option>
            </select>
            <p className={styles.fieldHint}>
              We'll personalize content based on your goals and experience
            </p>
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isSubmitting}
          >
            {isSubmitting ? 'Creating account...' : 'Sign Up'}
          </button>
        </form>

        <p className={styles.authActions}>
          <button onClick={onSwitchToPasswordReset} className={styles.linkButton}>
            Forgot password?
          </button>
        </p>

        <p className={styles.switchAuth}>
          Already have an account?{' '}
          <button onClick={onSwitchToSignin} className={styles.linkButton}>
            Sign in
          </button>
        </p>
      </div>
    </div>,
    document.body
  );
};
