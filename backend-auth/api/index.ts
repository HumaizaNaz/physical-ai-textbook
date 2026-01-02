// Avoid importing '@vercel/node' types (may not be installed); use plain types below.
import { Pool } from "pg";
import bcrypt from "bcryptjs";
import * as jwt from 'jsonwebtoken';

if (!process.env.DATABASE_URL) {
  throw new Error("DATABASE_URL is required");
}
if (!process.env.AUTH_SECRET) {
  throw new Error("AUTH_SECRET is required");
}

const JWT_SECRET = process.env.AUTH_SECRET as string;

// declare pgPool on globalThis so TypeScript knows the property exists
declare global {
  var pgPool: Pool | undefined;
}

let pool: Pool;

if (!globalThis.pgPool) {
  globalThis.pgPool = new Pool({
    connectionString: process.env.DATABASE_URL,
    ssl: { rejectUnauthorized: false }
  });
}

pool = globalThis.pgPool as Pool;

export default async function handler(req: any, res: any) {
  res.setHeader("Access-Control-Allow-Origin", "*");
  res.setHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
  res.setHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");

  if (req.method === "OPTIONS") return res.status(200).end();

  // Parse the URL to determine the route
  const url = new URL(req.url, `https://${req.headers.host}`);
  const path = url.pathname;
  const pathParts = path.split('/').filter(Boolean);

  try {
    // Handle root path
    if (path === '/' || path === '/api') {
      return res.status(200).json({ message: 'Backend Auth API is running!' });
    }

    // Handle test route
    if (path === '/api/test') {
      if (req.method === 'GET') {
        return res.status(200).json({ message: 'Server is running!' });
      }
    }

    // Handle health route
    if (path === '/api/health') {
      if (req.method === 'GET') {
        return res.status(200).json({ message: 'Auth service is healthy' });
      }
    }

    // Handle auth routes
    if (path.startsWith('/api/auth/')) {
      if (req.method === 'POST' && path === '/api/auth/sign-up/email') {
        const body = typeof req.body === 'string' ? JSON.parse(req.body) : req.body;
        const { email, password, name } = body;

        if (!email || !password || !name) {
          return res.status(400).json({ error: 'Email, password, and name are required' });
        }

        // Check if user already exists
        const existingUser = await pool.query(
          'SELECT id FROM users WHERE email = $1',
          [email]
        );

        if (existingUser.rows.length > 0) {
          return res.status(409).json({ error: 'User already exists' });
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

        return res.status(200).json({
          user: { id: user.id, email: user.email, name: user.name },
          access_token: token
        });
      }

      if (req.method === 'POST' && path === '/api/auth/sign-in/email') {
        const body = typeof req.body === 'string' ? JSON.parse(req.body) : req.body;
        const { email, password } = body;

        if (!email || !password) {
          return res.status(400).json({ error: 'Email and password are required' });
        }

        // Find user
        const result = await pool.query(
          'SELECT id, email, password, name FROM users WHERE email = $1',
          [email]
        );

        if (result.rows.length === 0) {
          return res.status(401).json({ error: 'Invalid email or password' });
        }

        const user = result.rows[0];

        // Verify password
        const isValid = await bcrypt.compare(password, user.password);
        if (!isValid) {
          return res.status(401).json({ error: 'Invalid email or password' });
        }

        // Create JWT token
        const token = jwt.sign(
          { userId: user.id, email: user.email },
          JWT_SECRET,
          { expiresIn: '7d' }
        );

        return res.status(200).json({
          user: { id: user.id, email: user.email, name: user.name },
          access_token: token
        });
      }

    }

    // Handle user profile routes
    if (path === '/api/user/me') {
      if (req.method === 'GET') {
        const authHeader = req.headers.authorization;

        if (!authHeader || !authHeader.startsWith('Bearer ')) {
          return res.status(401).json({ error: 'Authorization token required' });
        }

        const token = authHeader.substring(7);

        try {
          // Verify JWT token
          const decoded: any = jwt.verify(token, JWT_SECRET);
          const userId = decoded.userId;

          // Get user info
          const userResult = await pool.query(
            'SELECT id, email, name FROM users WHERE id = $1',
            [userId]
          );

          if (userResult.rows.length === 0) {
            return res.status(404).json({ error: 'User not found' });
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

          return res.status(200).json({
            ...user,
            ...background
          });
        } catch (error) {
          console.error('Profile fetch error:', error);
          return res.status(401).json({ error: 'Invalid token or profile fetch failed' });
        }
      }
    }

    if (path === '/api/user/background') {
      if (req.method === 'POST') {
        const authHeader = req.headers.authorization;

        if (!authHeader || !authHeader.startsWith('Bearer ')) {
          return res.status(401).json({ error: 'Authorization token required' });
        }

        const token = authHeader.substring(7);
        const body = typeof req.body === 'string' ? JSON.parse(req.body) : req.body;
        const profileData = body;

        try {
          // Verify JWT token
          const decoded: any = jwt.verify(token, JWT_SECRET);
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

          return res.status(200).json({ message: 'Profile updated successfully' });
        } catch (error) {
          console.error('Profile update error:', error);
          return res.status(500).json({ error: 'Profile update failed' });
        }
      }
    }

    // Handle auth routes
    if (path.startsWith('/auth/')) {
      if (req.method === 'POST' && path === '/auth/sign-up/email') {
        const body = typeof req.body === 'string' ? JSON.parse(req.body) : req.body;
        const { email, password, name } = body;

        if (!email || !password || !name) {
          return res.status(400).json({ error: 'Email, password, and name are required' });
        }

        // Check if user already exists
        const existingUser = await pool.query(
          'SELECT id FROM users WHERE email = $1',
          [email]
        );

        if (existingUser.rows.length > 0) {
          return res.status(409).json({ error: 'User already exists' });
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

        return res.status(200).json({
          user: { id: user.id, email: user.email, name: user.name },
          access_token: token
        });
      }

      if (req.method === 'POST' && path === '/auth/sign-in/email') {
        const body = typeof req.body === 'string' ? JSON.parse(req.body) : req.body;
        const { email, password } = body;

        if (!email || !password) {
          return res.status(400).json({ error: 'Email and password are required' });
        }

        // Find user
        const result = await pool.query(
          'SELECT id, email, password, name FROM users WHERE email = $1',
          [email]
        );

        if (result.rows.length === 0) {
          return res.status(401).json({ error: 'Invalid email or password' });
        }

        const user = result.rows[0];

        // Verify password
        const isValid = await bcrypt.compare(password, user.password);
        if (!isValid) {
          return res.status(401).json({ error: 'Invalid email or password' });
        }

        // Create JWT token
        const token = jwt.sign(
          { userId: user.id, email: user.email },
          JWT_SECRET,
          { expiresIn: '7d' }
        );

        return res.status(200).json({
          user: { id: user.id, email: user.email, name: user.name },
          access_token: token
        });
      }

      // Password reset request endpoint
      if (req.method === 'POST' && path === '/auth/reset-password/request') {
        const body = typeof req.body === 'string' ? JSON.parse(req.body) : req.body;
        const { email } = body;

        if (!email) {
          return res.status(400).json({ error: 'Email is required' });
        }

        try {
          // Check if user exists
          const userResult = await pool.query(
            'SELECT id, email FROM users WHERE email = $1',
            [email]
          );

          if (userResult.rows.length === 0) {
            // For security, return success even if email doesn't exist to prevent email enumeration
            return res.status(200).json({
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

          return res.status(200).json({
            message: 'If an account with this email exists, password reset instructions have been sent'
          });
        } catch (error) {
          console.error('Password reset request error:', error);
          return res.status(500).json({ error: 'Password reset request failed' });
        }
      }

      // Password reset endpoint
      if (req.method === 'POST' && path === '/auth/reset-password') {
        const body = typeof req.body === 'string' ? JSON.parse(req.body) : req.body;
        const { token, newPassword } = body;

        if (!token || !newPassword) {
          return res.status(400).json({ error: 'Token and new password are required' });
        }

        try {
          // Verify the reset token
          const decoded: any = jwt.verify(token, JWT_SECRET);

          if (decoded.type !== 'password_reset') {
            return res.status(400).json({ error: 'Invalid token type' });
          }

          // Hash the new password
          const hashedPassword = await bcrypt.hash(newPassword, 10);

          // Update the user's password
          await pool.query(
            'UPDATE users SET password = $1 WHERE email = $2',
            [hashedPassword, decoded.email]
          );

          return res.status(200).json({
            message: 'Password reset successfully'
          });
        } catch (error) {
          return res.status(400).json({ error: 'Invalid or expired token' });
        }
      }
    }

    // If no route matches
    return res.status(404).json({ error: 'Route not found' });
  } catch (error: any) {
    console.error('API Error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
}

export const config = {
  api: {
    externalResolver: true,
  },
};