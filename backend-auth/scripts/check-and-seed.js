// Script to manually add test data to the database using direct SQL
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

async function checkAndSeedDatabase() {
  try {
    console.log('Checking database tables...');
    
    // Check if users_background table exists
    const result = await pool.query(`
      SELECT column_name, data_type 
      FROM information_schema.columns 
      WHERE table_name = 'users_background'
    `);
    
    console.log('users_background table columns:', result.rows);
    
    // Try to insert test data
    const testUserId = uuidv4(); // Generate a UUID
    
    try {
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
    } catch (insertError) {
      console.log('Insert failed, trying to create table manually...');
      // Create the table if it doesn't exist properly
      await pool.query(`
        CREATE TABLE IF NOT EXISTS users_background (
          user_id UUID PRIMARY KEY,
          programming_level TEXT NOT NULL,
          hardware_level TEXT NOT NULL
        );
      `);
      
      // Now try to insert again
      const insertResult = await pool.query(
        `INSERT INTO users_background (user_id, programming_level, hardware_level) 
         VALUES ($1, $2, $3) 
         ON CONFLICT (user_id) DO NOTHING
         RETURNING *`,
        [testUserId, 'Intermediate', 'Basic']
      );
      
      console.log('Test data added successfully after table creation!');
      console.log('Added user background:', insertResult.rows[0]);
    }
    
    // Close the connection
    await pool.end();
  } catch (error) {
    console.error('Error checking/seeding database:', error);
    await pool.end();
    process.exit(1);
  }
}

checkAndSeedDatabase();