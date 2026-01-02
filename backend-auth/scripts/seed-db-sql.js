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

async function seedDatabase() {
  try {
    console.log('Seeding database with test data...');
    
    // Add some test user background data
    const testUserId = uuidv4(); // Generate a UUID
    
    const result = await pool.query(
      `INSERT INTO users_background (user_id, programming_level, hardware_level) 
       VALUES ($1, $2, $3) 
       RETURNING *`,
      [testUserId, 'Intermediate', 'Basic']
    );
    
    console.log('Test data added successfully!');
    console.log('Added user background:', result.rows[0]);
    
    // Close the connection
    await pool.end();
  } catch (error) {
    console.error('Error seeding database:', error);
    await pool.end();
    process.exit(1);
  }
}

seedDatabase();