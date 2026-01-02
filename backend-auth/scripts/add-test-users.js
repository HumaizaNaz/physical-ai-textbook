// Script to add test user accounts to the database
import 'dotenv/config';
import { Pool } from 'pg';
import { v4 as uuidv4 } from 'uuid';
import { sha256 } from 'crypto-hash';

const DATABASE_URL = process.env.DATABASE_URL;

if (!DATABASE_URL) {
  console.error('ERROR: DATABASE_URL not set.');
  process.exit(1);
}

const pool = new Pool({ 
  connectionString: DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  }
});

async function addTestUsers() {
  try {
    console.log('Adding test user accounts to the database...');
    
    // Check if user table exists
    const userTableCheck = await pool.query(`
      SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_name = 'user'
      );
    `);
    
    if (!userTableCheck.rows[0].exists) {
      console.log('Better Auth user table does not exist yet. Users will be created through the auth system.');
      console.log('You need to register users through the frontend or API first.');
      await pool.end();
      return;
    }
    
    console.log('User table exists. Checking its structure...');
    
    // Check user table structure
    const userColumns = await pool.query(`
      SELECT column_name, data_type 
      FROM information_schema.columns 
      WHERE table_name = 'user'
      ORDER BY ordinal_position;
    `);
    
    console.log('User table columns:', userColumns.rows.map(col => col.column_name));
    
    // Create test users through the proper Better Auth flow would require the full auth system
    // Instead, let me show you how to register users through the API
    console.log('\nTo create test users, you can:');
    console.log('1. Use the frontend signup form');
    console.log('2. Or make API calls like:');
    console.log('   POST /api/auth/sign-up/email with body: {"email":"test@example.com","password":"password123"}');
    
    // Show existing users if any
    const existingUsers = await pool.query('SELECT id, email, name FROM "user" LIMIT 5;');
    if (existingUsers.rows.length > 0) {
      console.log('\nExisting users in database:');
      existingUsers.rows.forEach(user => {
        console.log(`- ID: ${user.id}, Email: ${user.email}, Name: ${user.name || 'N/A'}`);
      });
    } else {
      console.log('\nNo existing users found. Users will be created when they register.');
    }
    
    await pool.end();
  } catch (error) {
    console.error('Error adding test users:', error);
    await pool.end();
    process.exit(1);
  }
}

addTestUsers();