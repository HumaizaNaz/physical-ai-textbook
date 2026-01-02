// Script to manually add test data to the database
import 'dotenv/config';
import { drizzle } from 'drizzle-orm/node-postgres';
import { Pool } from 'pg';
import { users_background } from '../src/schema.js'; // Import your custom schema
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

const db = drizzle(pool);

async function seedDatabase() {
  try {
    console.log('Seeding database with test data...');
    
    // Add some test user background data
    const testUserId = uuidv4(); // Generate a UUID
    
    await db.insert(users_background).values({
      user_id: testUserId,
      programming_level: 'Intermediate',
      hardware_level: 'Basic'
    });
    
    console.log('Test data added successfully!');
    console.log('Added user background with ID:', testUserId);
    
    // Close the connection
    await pool.end();
  } catch (error) {
    console.error('Error seeding database:', error);
    await pool.end();
    process.exit(1);
  }
}

seedDatabase();