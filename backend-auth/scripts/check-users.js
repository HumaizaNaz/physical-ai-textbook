// Script to check existing users and show how to add test users
import 'dotenv/config';
import { Pool } from 'pg';
import { v4 as uuidv4 } from 'uuid';

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

async function checkUsers() {
  try {
    console.log('Checking existing user accounts in the database...');
    
    // Check if user table exists
    const userTableCheck = await pool.query(`
      SELECT EXISTS (
        SELECT FROM information_schema.tables 
        WHERE table_name = 'user'
      );
    `);
    
    if (!userTableCheck.rows[0].exists) {
      console.log('Better Auth user table does not exist yet.');
      console.log('The table will be created automatically when the auth system runs.');
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
    
    // Show existing users if any
    const existingUsers = await pool.query('SELECT id, email, name, created_at FROM "user" LIMIT 10;');
    if (existingUsers.rows.length > 0) {
      console.log('\nExisting users in database:');
      existingUsers.rows.forEach(user => {
        console.log(`- ID: ${user.id}, Email: ${user.email}, Name: ${user.name || 'N/A'}, Created: ${user.created_at}`);
      });
    } else {
      console.log('\nNo existing users found in the database.');
      console.log('Users will be created when they register through the frontend.');
    }
    
    // Show how to create test users via API
    console.log('\nTo create test users, you can register them using the frontend or API:');
    console.log('Frontend: Use the signup form at your website');
    console.log('API: POST /api/auth/sign-up/email with JSON body:');
    console.log('  { "email": "test@example.com", "password": "password123" }');
    
    console.log('\nSome example test emails you can use:');
    const testEmails = [
      'test1@example.com',
      'test2@example.com', 
      'developer@test.com',
      'hardware@test.com',
      'admin@test.com'
    ];
    
    testEmails.forEach(email => {
      console.log(`- ${email}`);
    });
    
    await pool.end();
  } catch (error) {
    console.error('Error checking users:', error);
    await pool.end();
    process.exit(1);
  }
}

checkUsers();