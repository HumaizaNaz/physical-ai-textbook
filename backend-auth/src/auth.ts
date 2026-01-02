// backend-auth/src/auth.ts
import bcrypt from 'bcrypt';
import { Pool } from 'pg';
import 'dotenv/config';

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: { rejectUnauthorized: false }
});

// Simple authentication functions
export async function authenticateUser(email: string, password: string) {
  try {
    // Find user in the database
    const result = await pool.query(
      'SELECT id, email, password, name FROM "user" WHERE email = $1',
      [email]
    );

    if (result.rows.length === 0) {
      throw new Error('Invalid email or password');
    }

    const user = result.rows[0];

    // Verify password
    const isValid = await bcrypt.compare(password, user.password);
    if (!isValid) {
      throw new Error('Invalid email or password');
    }

    // Return user data (without password)
    return {
      id: user.id,
      email: user.email,
      name: user.name
    };
  } catch (error) {
    throw error;
  }
}

export async function createUser(email: string, password: string, name: string) {
  try {
    // Hash the password
    const hashedPassword = await bcrypt.hash(password, 10);

    // Check if user already exists
    const existingUser = await pool.query(
      'SELECT id FROM "user" WHERE email = $1',
      [email]
    );

    if (existingUser.rows.length > 0) {
      throw new Error('User already exists');
    }

    // Create user
    const result = await pool.query(
      `INSERT INTO "user" (id, email, password, name, created_at, updated_at) 
       VALUES (gen_random_uuid(), $1, $2, $3, NOW(), NOW()) 
       RETURNING id, email, name`,
      [email, hashedPassword, name]
    );

    return result.rows[0];
  } catch (error) {
    throw error;
  }
}

export async function getUserProfile(userId: string) {
  try {
    // Get user info
    const userResult = await pool.query(
      'SELECT id, email, name FROM "user" WHERE id = $1',
      [userId]
    );

    if (userResult.rows.length === 0) {
      throw new Error('User not found');
    }

    const user = userResult.rows[0];

    // Get user background
    const backgroundResult = await pool.query(
      `SELECT 
         programming_experience, 
         ros_experience, 
         linux_familiarity, 
         hardware_experience, 
         electronics_knowledge, 
         robotics_projects, 
         learning_goal
       FROM users_background 
       WHERE user_id = $1`,
      [userId]
    );

    const background = backgroundResult.rows[0] || {
      programming_experience: 'None',
      ros_experience: 'None',
      linux_familiarity: 'Beginner',
      hardware_experience: 'None',
      electronics_knowledge: 'None',
      robotics_projects: 'None',
      learning_goal: 'General Learning'
    };

    return {
      ...user,
      ...background
    };
  } catch (error) {
    throw error;
  }
}

export async function updateUserProfile(userId: string, profileData: any) {
  try {
    // Update or insert user background
    await pool.query(`
      INSERT INTO users_background (
        user_id, 
        programming_experience, 
        ros_experience, 
        linux_familiarity, 
        hardware_experience, 
        electronics_knowledge, 
        robotics_projects, 
        learning_goal
      ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
      ON CONFLICT (user_id) 
      DO UPDATE SET
        programming_experience = EXCLUDED.programming_experience,
        ros_experience = EXCLUDED.ros_experience,
        linux_familiarity = EXCLUDED.linux_familiarity,
        hardware_experience = EXCLUDED.hardware_experience,
        electronics_knowledge = EXCLUDED.electronics_knowledge,
        robotics_projects = EXCLUDED.robotics_projects,
        learning_goal = EXCLUDED.learning_goal
    `, [
      userId,
      profileData.programming_experience || 'None',
      profileData.ros_experience || 'None',
      profileData.linux_familiarity || 'Beginner',
      profileData.hardware_experience || 'None',
      profileData.electronics_knowledge || 'None',
      profileData.robotics_projects || 'None',
      profileData.learning_goal || 'General Learning'
    ]);

    return { success: true };
  } catch (error) {
    throw error;
  }
}