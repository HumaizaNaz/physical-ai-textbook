import React, { useState, useEffect } from 'react';
import { createPortal } from 'react-dom';
import { useAuth } from './AuthContext';
import styles from './styles.module.css';

interface ProfileSettingsModalProps {
  isOpen: boolean;
  onClose: () => void;
}

export const ProfileSettingsModal: React.FC<ProfileSettingsModalProps> = ({ isOpen, onClose }) => {
  const { user, updateProfile } = useAuth();

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

  const [error, setError] = useState('');
  const [successMessage, setSuccessMessage] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);

  // Pre-fill current user's profile
  useEffect(() => {
    if (user) {
      setProgrammingExperience(user.programming_experience || 'None');
      setRosExperience(user.ros_experience || 'None');
      setLinuxFamiliarity(user.linux_familiarity || 'Beginner');
      setHardwareExperience(user.hardware_experience || 'None');
      setElectronicsKnowledge(user.electronics_knowledge || 'None');
      setRoboticsProjects(user.robotics_projects || 'None');
      setLearningGoal(user.learning_goal || 'General Learning');
    }
  }, [user]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setSuccessMessage('');
    setIsSubmitting(true);

    try {
      await updateProfile({
        programming_experience: programmingExperience,
        ros_experience: rosExperience,
        linux_familiarity: linuxFamiliarity,
        hardware_experience: hardwareExperience,
        electronics_knowledge: electronicsKnowledge,
        robotics_projects: roboticsProjects,
        learning_goal: learningGoal
      });
      setSuccessMessage('Profile updated successfully!');
      setTimeout(() => {
        setSuccessMessage('');
        onClose();
      }, 2000);
    } catch (err: any) {
      setError(err.message || 'Update failed. Please try again.');
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleClose = () => {
    setError('');
    setSuccessMessage('');
    onClose();
  };

  if (!isOpen || !user) return null;
  if (typeof document === 'undefined') return null;

  // Check if anything has changed
  const hasChanges =
    programmingExperience !== (user.programming_experience || 'None') ||
    rosExperience !== (user.ros_experience || 'None') ||
    linuxFamiliarity !== (user.linux_familiarity || 'Beginner') ||
    hardwareExperience !== (user.hardware_experience || 'None') ||
    electronicsKnowledge !== (user.electronics_knowledge || 'None') ||
    roboticsProjects !== (user.robotics_projects || 'None') ||
    learningGoal !== (user.learning_goal || 'General Learning');

  return createPortal(
    <div className={styles.modalOverlay} onClick={handleClose}>
      <div className={`${styles.modalContent} ${styles.scrollableModal}`} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={handleClose} aria-label="Close">
          ×
        </button>

        <h2 className={styles.modalTitle}>Profile Settings</h2>
        <p className={styles.modalSubtitle}>Update your background and preferences</p>

        {error && <div className={styles.errorMessage}>{error}</div>}
        {successMessage && <div className={styles.successMessage}>{successMessage}</div>}

        <form onSubmit={handleSubmit} className={styles.authForm}>
          <div className={styles.formGroup}>
            <label htmlFor="profile-email">Email</label>
            <input
              id="profile-email"
              type="email"
              value={user.email}
              disabled
              className={styles.disabledInput}
            />
            <p className={styles.fieldHint}>Email cannot be changed</p>
          </div>

          {/* Software Background Section */}
          <div className={styles.sectionDivider}>
            <span className={styles.sectionTitle}>Software Background</span>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="profile-programming">Programming Experience</label>
            <select
              id="profile-programming"
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
            <label htmlFor="profile-ros">ROS Experience</label>
            <select
              id="profile-ros"
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
            <label htmlFor="profile-linux">Linux Familiarity</label>
            <select
              id="profile-linux"
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
            <label htmlFor="profile-hardware">Hardware Experience</label>
            <select
              id="profile-hardware"
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
            <label htmlFor="profile-electronics">Electronics Knowledge</label>
            <select
              id="profile-electronics"
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
            <label htmlFor="profile-projects">Robotics Projects Completed</label>
            <select
              id="profile-projects"
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
            <label htmlFor="profile-goal">Primary Learning Goal</label>
            <select
              id="profile-goal"
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
              Content will be personalized based on your preferences
            </p>
          </div>

          <div className={styles.profileInfo}>
            <p><strong>Account Created:</strong> {new Date(user.created_at).toLocaleDateString()}</p>
            <p><strong>Last Updated:</strong> {new Date(user.updated_at).toLocaleDateString()}</p>
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isSubmitting || !hasChanges}
          >
            {isSubmitting ? 'Saving...' : 'Save Changes'}
          </button>
        </form>
      </div>
    </div>,
    document.body
  );
};
