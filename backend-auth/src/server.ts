// backend-auth/src/server.ts
import 'dotenv/config';
import { serve } from '@hono/node-server';
import { Hono } from 'hono';
import { cors } from 'hono/cors';
import { Pool } from 'pg';
import * as bcrypt from 'bcryptjs';
import jwt from 'jsonwebtoken';

const app = new Hono();
const PORT = process.env.PORT ? parseInt(process.env.PORT, 10) : 7860;
const FRONTEND_URL = process.env.FRONTEND_URL || 'http://localhost:3000';
const JWT_SECRET = process.env.AUTH_SECRET || 'fallback_secret_for_development';

const DATABASE_URL = process.env.DATABASE_URL;
if (!DATABASE_URL) {
  console.error('ERROR: DATABASE_URL not set.');
  process.exit(1);
}

const pool = new Pool({
  connectionString: DATABASE_URL,
  ssl: { rejectUnauthorized: false }
});

// --- Middleware ---
app.use('*', cors({ origin: FRONTEND_URL, credentials: true }));

// --- Test Route ---
app.get('/test', (c) => {
  return c.text('Server is running!');
});

// --- Authentication Endpoints ---

// Sign Up
app.post('/auth/sign-up/email', async (c) => {
  const { email, password, name } = await c.req.json();

  if (!email || !password || !name) {
    return c.json({ error: 'Email, password, and name are required' }, 400);
  }

  try {
    // Check if user already exists
    const existingUser = await pool.query(
      'SELECT id FROM users WHERE email = $1',
      [email]
    );

    if (existingUser.rows.length > 0) {
      return c.json({ error: 'User already exists' }, 409);
    }

    // Hash the password
    const hashedPassword = await bcrypt.hash(password, 10);

    // Create user
    const result = await pool.query(
      `INSERT INTO users (id, email, password, name, created_at, updated_at)
       VALUES (gen_random_uuid(), $1, $2, $3, NOW(), NOW())
       RETURNING id, email, name`,
      [email, hashedPassword, name]
    );

    const user = result.rows[0];

    // Create JWT token
    const token = jwt.sign(
      { userId: user.id, email: user.email },
      JWT_SECRET,
      { expiresIn: '7d' }
    );

    return c.json({
      user: { id: user.id, email: user.email, name: user.name },
      access_token: token
    });
  } catch (error: any) {
    console.error('Signup error:', error);
    return c.json({ error: error.message || 'Signup failed' }, 500);
  }
});

// Sign In
app.post('/auth/sign-in/email', async (c) => {
  const { email, password } = await c.req.json();

  if (!email || !password) {
    return c.json({ error: 'Email and password are required' }, 400);
  }

  try {
    // Find user
    const result = await pool.query(
      'SELECT id, email, password, name FROM users WHERE email = $1',
      [email]
    );

    if (result.rows.length === 0) {
      return c.json({ error: 'Invalid email or password' }, 401);
    }

    const user = result.rows[0];

    // Verify password
    const isValid = await bcrypt.compare(password, user.password);
    if (!isValid) {
      return c.json({ error: 'Invalid email or password' }, 401);
    }

    // Create JWT token
    const token = jwt.sign(
      { userId: user.id, email: user.email },
      JWT_SECRET,
      { expiresIn: '7d' }
    );

    return c.json({
      user: { id: user.id, email: user.email, name: user.name },
      access_token: token
    });
  } catch (error: any) {
    console.error('Signin error:', error);
    return c.json({ error: error.message || 'Signin failed' }, 500);
  }
});

// Get User Profile
app.get('/user/me', async (c) => {
  const authHeader = c.req.header('Authorization');

  if (!authHeader || !authHeader.startsWith('Bearer ')) {
    return c.json({ error: 'Authorization token required' }, 401);
  }

  const token = authHeader.substring(7);

  try {
    // Verify JWT token
    const decoded = jwt.verify(token, JWT_SECRET) as { userId: string; email: string };
    const userId = decoded.userId;

    // Get user info
    const userResult = await pool.query(
      'SELECT id, email, name FROM users WHERE id = $1',
      [userId]
    );

    if (userResult.rows.length === 0) {
      return c.json({ error: 'User not found' }, 404);
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

    return c.json({
      ...user,
      ...background
    });
  } catch (error) {
    console.error('Profile fetch error:', error);
    return c.json({ error: 'Invalid token or profile fetch failed' }, 401);
  }
});

// Update User Profile
app.post('/user/background', async (c) => {
  const authHeader = c.req.header('Authorization');

  if (!authHeader || !authHeader.startsWith('Bearer ')) {
    return c.json({ error: 'Authorization token required' }, 401);
  }

  const token = authHeader.substring(7);
  const profileData = await c.req.json();

  try {
    // Verify JWT token
    const decoded = jwt.verify(token, JWT_SECRET) as { userId: string; email: string };
    const userId = decoded.userId;

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

    return c.json({ message: 'Profile updated successfully' });
  } catch (error) {
    console.error('Profile update error:', error);
    return c.json({ error: 'Profile update failed' }, 500);
  }
});

// Password reset request endpoint
app.post('/auth/reset-password/request', async (c) => {
  const { email } = await c.req.json();

  if (!email) {
    return c.json({ error: 'Email is required' }, 400);
  }

  try {
    // Check if user exists
    const userResult = await pool.query(
      'SELECT id, email FROM users WHERE email = $1',
      [email]
    );

    if (userResult.rows.length === 0) {
      // For security, return success even if email doesn't exist to prevent email enumeration
      return c.json({
        message: 'If an account with this email exists, password reset instructions have been sent'
      });
    }

    // Generate a reset token
    const resetToken = jwt.sign(
      { email, type: 'password_reset' },
      JWT_SECRET,
      { expiresIn: '1h' } // Token expires in 1 hour
    );

    // In a real application, you would send an email with the reset token
    // For now, we'll just log it for testing purposes
    console.log(`Password reset token for ${email}: ${resetToken}`);

    // In production, you would send an email here using a service like SendGrid, AWS SES, etc.
    // Example with nodemailer:
    /*
    await sendPasswordResetEmail(email, resetToken);
    */

    return c.json({
      message: 'If an account with this email exists, password reset instructions have been sent'
    });
  } catch (error) {
    console.error('Password reset request error:', error);
    return c.json({ error: 'Password reset request failed' }, 500);
  }
});

// Password reset endpoint
app.post('/auth/reset-password', async (c) => {
  const { token, newPassword } = await c.req.json();

  if (!token || !newPassword) {
    return c.json({ error: 'Token and new password are required' }, 400);
  }

  try {
    // Verify the reset token
    const decoded = jwt.verify(token, JWT_SECRET) as { email: string; type: string };

    if (decoded.type !== 'password_reset') {
      return c.json({ error: 'Invalid token type' }, 400);
    }

    // Hash the new password
    const hashedPassword = await bcrypt.hash(newPassword, 10);

    // Update the user's password
    await pool.query(
      'UPDATE users SET password = $1 WHERE email = $2',
      [hashedPassword, decoded.email]
    );

    return c.json({
      message: 'Password reset successfully'
    });
  } catch (error) {
    console.error('Password reset error:', error);
    if ((error as Error).message.includes('invalid token') || (error as Error).message.includes('jwt expired')) {
      return c.json({ error: 'Invalid or expired token' }, 400);
    }
    return c.json({ error: 'Password reset failed' }, 500);
  }
});

// Health check
app.get('/health', (c) => {
  return c.text('Auth service is healthy');
});

console.log(`Auth server running on http://localhost:${PORT}`);

serve({
  fetch: app.fetch,
  port: PORT,
});