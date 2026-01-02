// Script to manually add test data to the database using existing table structure
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

async function seedDatabase() {
  try {
    console.log('Seeding database with test data using existing structure...');
    
    // Check if users_background table exists and get its structure
    const result = await pool.query(`
      SELECT column_name, data_type 
      FROM information_schema.columns 
      WHERE table_name = 'users_background'
      ORDER BY ordinal_position;
    `);
    
    console.log('users_background table columns:', result.rows.map(col => col.column_name));
    
    // Check if we need to update the column name
    const hasHardwareLevel = result.rows.some(col => col.column_name === 'hardware_level');
    const hasHardwareExperience = result.rows.some(col => col.column_name === 'hardware_experience');
    
    if (hasHardwareExperience && !hasHardwareLevel) {
      console.log('Renaming hardware_experience to hardware_level...');
      await pool.query('ALTER TABLE users_background RENAME COLUMN hardware_experience TO hardware_level;');
      console.log('Column renamed successfully');
    }
    
    // Now insert test data
    const testUserId = uuidv4(); // Generate a UUID
    
    const insertResult = await pool.query(
      `INSERT INTO users_background (user_id, programming_level, hardware_level) 
       VALUES ($1, $2, $3) 
       ON CONFLICT (user_id) DO NOTHING
       RETURNING *`,
      [testUserId, 'Intermediate', 'Basic']
    );
    
    if (insertResult.rows.length > 0) {
      console.log('Test data added successfully!');
      console.log('Added user background:', insertResult.rows[0]);
    } else {
      console.log('Test data inserted (or already existed)');
    }
    
    // Also try to remove the extra 'id' column if it exists
    const hasIdColumn = result.rows.some(col => col.column_name === 'id');
    if (hasIdColumn) {
      console.log('Removing extra id column...');
      await pool.query('ALTER TABLE users_background DROP COLUMN IF EXISTS id;');
      console.log('Extra id column removed');
    }
    
    // Close the connection
    await pool.end();
  } catch (error) {
    console.error('Error seeding database:', error);
    await pool.end();
    process.exit(1);
  }
}

seedDatabase();