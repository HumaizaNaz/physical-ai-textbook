// api/auth.ts for Vercel
import { Hono } from 'hono';
import { cors } from 'hono/cors';
import { Pool } from 'pg';
import * as bcrypt from 'bcrypt';
import * as jwt from 'jsonwebtoken';

const app = new Hono();

// Initialize database connection
const DATABASE_URL = process.env.DATABASE_URL;
if (!DATABASE_URL) {
  throw new Error('DATABASE_URL is not set');
}

const pool = new Pool({
  connectionString: DATABASE_URL,
  ssl: process.env.NODE_ENV === 'production' ? { rejectUnauthorized: false } : undefined
});

const JWT_SECRET = process.env.AUTH_SECRET || 'fallback_secret_for_development';

// CORS middleware
app.use('*', cors());

// Test route
app.get('/test', (c) => {
  return c.text('Server is running!');
});

// Health check
app.get('/health', (c) => {
  return c.text('Auth service is healthy');
});

// Sign Up
app.post('/auth/sign-up/email', async (c) => {
  try {
    const body = await c.req.json();
    const { email, password, name } = body;

    if (!email || !password || !name) {
      return c.json({ error: 'Email, password, and name are required' }, 400);
    }

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
    if (error.message.includes('Unexpected end of JSON input')) {
      return c.json({ error: 'Invalid JSON in request body' }, 400);
    }
    console.error('Signup error:', error);
    return c.json({ error: error.message || 'Signup failed' }, 500);
  }
});

// Sign In
app.post('/auth/sign-in/email', async (c) => {
  try {
    const body = await c.req.json();
    const { email, password } = body;

    if (!email || !password) {
      return c.json({ error: 'Email and password are required' }, 400);
    }

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
    if (error.message.includes('Unexpected end of JSON input')) {
      return c.json({ error: 'Invalid JSON in request body' }, 400);
    }
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
  } catch (error: any) {
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
  
  try {
    const profileData = await c.req.json();

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
  } catch (error: any) {
    if (error.message.includes('Unexpected end of JSON input')) {
      return c.json({ error: 'Invalid JSON in request body' }, 400);
    }
    console.error('Profile update error:', error);
    return c.json({ error: 'Profile update failed' }, 500);
  }
});

export default app;