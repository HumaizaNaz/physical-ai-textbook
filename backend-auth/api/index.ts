// Avoid importing '@vercel/node' types (may not be installed); use plain types below.
import { Pool } from "pg";
import bcrypt from "bcryptjs";
import * as jwt from 'jsonwebtoken';

const DATABASE_URL = process.env.DATABASE_URL;
const AUTH_SECRET = process.env.AUTH_SECRET;

if (!DATABASE_URL) {
  console.error("DATABASE_URL is not set");
  throw new Error("DATABASE_URL is required");
}
if (!AUTH_SECRET) {
  console.error("AUTH_SECRET is not set");
  throw new Error("AUTH_SECRET is required");
}

const JWT_SECRET = AUTH_SECRET || "fallback_secret_for_development_which_should_not_be_used_in_production";

// declare pgPool on globalThis so TypeScript knows the property exists
declare global {
  var pgPool: Pool | undefined;
}

let pool: Pool | null = null;

if (DATABASE_URL) {
  if (!globalThis.pgPool) {
    globalThis.pgPool = new Pool({
      connectionString: DATABASE_URL,
      ssl: { rejectUnauthorized: false }
    });
  }
  pool = globalThis.pgPool as Pool;
} else {
  console.error("Database connection not available - DATABASE_URL is not set");
}

export default async function handler(req: any, res: any) {
  // Check if required environment variables are set
  if (!AUTH_SECRET) {
    console.error("AUTH_SECRET is not set - using fallback (not recommended for production)");
  }

  if (!pool) {
    console.error("Database connection not available");
    return res.status(500).json({ error: 'Server configuration error: Database connection not available' });
  }

  // CORS headers - Allow your frontend domain specifically
  const allowedOrigins = [
    'https://physical-ai-textbook-five.vercel.app',
    'http://localhost:3000',
    'http://localhost:3001',
    process.env.FRONTEND_URL
  ].filter(Boolean); // Remove any undefined values

  const origin = req.headers.origin;

  // Check if the origin is in the allowed list
  if (origin && allowedOrigins.includes(origin)) {
    res.setHeader("Access-Control-Allow-Origin", origin);
  } else {
    // Default to the primary frontend URL if origin is not in the allowed list
    res.setHeader("Access-Control-Allow-Origin", 'https://physical-ai-textbook-five.vercel.app');
  }

  res.setHeader("Access-Control-Allow-Credentials", "true");
  res.setHeader("Access-Control-Allow-Headers", "Content-Type, Authorization, X-Requested-With, Accept, Origin, Access-Control-Request-Method, Access-Control-Request-Headers");
  res.setHeader("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE,OPTIONS,PATCH");
  res.setHeader("Access-Control-Max-Age", "86400"); // 24 hours

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
      // Check if database connection is available
      if (!pool) {
        return res.status(500).json({ error: 'Database connection not available' });
      }

      // Sign up
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

      // Sign in
      if (req.method === 'POST' && path === '/api/auth/sign-in/email') {
        // Check if database connection is available
        if (!pool) {
          return res.status(500).json({ error: 'Database connection not available' });
        }

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
      if (req.method === 'POST' && path === '/api/auth/reset-password/request') {
        // Check if database connection is available
        if (!pool) {
          return res.status(500).json({ error: 'Database connection not available' });
        }

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
      if (req.method === 'POST' && path === '/api/auth/reset-password') {
        // Check if database connection is available
        if (!pool) {
          return res.status(500).json({ error: 'Database connection not available' });
        }

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

    // Handle user profile routes
    if (path === '/api/user/me') {
      if (req.method === 'GET') {
        // Check if database connection is available
        if (!pool) {
          return res.status(500).json({ error: 'Database connection not available' });
        }

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
        // Check if database connection is available
        if (!pool) {
          return res.status(500).json({ error: 'Database connection not available' });
        }

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

    // Password reset request endpoint
    if (req.method === 'POST' && path === '/api/auth/reset-password/request') {
      const body = typeof req.body === 'string' ? JSON.parse(req.body) : req.body;
      const { email } = body;

      if (!email) {
        return res.status(400).json({ error: 'Email is required' });
      }

      // Validate email format
      const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
      if (!emailRegex.test(email)) {
        return res.status(400).json({ error: 'Invalid email format' });
      }

      try {
        // Check if user exists
        const userResult = await pool.query(
          'SELECT id, email FROM users WHERE email = $1',
          [email]
        );

        if (userResult.rows.length === 0) {
          // For security, return success even if email doesn't exist to prevent email enumeration
          console.log(`Password reset requested for non-existent email: ${email}`);
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
        console.log(`Password reset token generated for ${email}: ${resetToken}`);

        // In production, you would send an email here using a service like SendGrid, AWS SES, etc.
        // Example with nodemailer:
        /*
        await sendPasswordResetEmail(email, resetToken);
        */

        return res.status(200).json({
          message: 'If an account with this email exists, password reset instructions have been sent'
        });
      } catch (error: any) {
        console.error('Password reset request error:', error);
        console.error('Error details:', error.message, error.stack);
        return res.status(500).json({
          error: 'Password reset request failed',
          details: process.env.NODE_ENV === 'development' ? error.message : undefined
        });
      }
    }

    // Password reset endpoint
    if (req.method === 'POST' && path === '/api/auth/reset-password') {
      const body = typeof req.body === 'string' ? JSON.parse(req.body) : req.body;
      const { token, newPassword } = body;

      if (!token || !newPassword) {
        return res.status(400).json({ error: 'Token and new password are required' });
      }

      // Validate password strength
      if (newPassword.length < 8) {
        return res.status(400).json({ error: 'Password must be at least 8 characters long' });
      }

      try {
        // Verify the reset token
        const decoded: any = jwt.verify(token, JWT_SECRET);

        if (decoded.type !== 'password_reset') {
          return res.status(400).json({ error: 'Invalid token type' });
        }

        if (!decoded.email) {
          return res.status(400).json({ error: 'Email not found in token' });
        }

        // Hash the new password
        const hashedPassword = await bcrypt.hash(newPassword, 10);

        // Update the user's password
        const result = await pool.query(
          'UPDATE users SET password = $1 WHERE email = $2 RETURNING id',
          [hashedPassword, decoded.email]
        );

        if (result.rowCount === 0) {
          return res.status(400).json({ error: 'User not found' });
        }

        console.log(`Password successfully reset for user: ${decoded.email}`);

        return res.status(200).json({
          message: 'Password reset successfully'
        });
      } catch (error: any) {
        console.error('Password reset error:', error);
        if (error.name === 'TokenExpiredError') {
          return res.status(400).json({ error: 'Reset token has expired' });
        } else if (error.name === 'JsonWebTokenError') {
          return res.status(400).json({ error: 'Invalid reset token' });
        }
        return res.status(400).json({
          error: 'Invalid or expired token',
          details: process.env.NODE_ENV === 'development' ? error.message : undefined
        });
      }
    }

    // If no route matches
    return res.status(404).json({ error: 'Route not found' });
  } catch (error: any) {
    console.error('API Error:', error);
    console.error('Error details:', {
      message: error.message,
      stack: error.stack,
      name: error.name
    });
    return res.status(500).json({
      error: 'Internal server error',
      details: process.env.NODE_ENV === 'development' ? error.message : undefined
    });
  }
}

export const config = {
  api: {
    externalResolver: true,
  },
};