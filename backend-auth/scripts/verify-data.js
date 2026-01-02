// Verify the database table and data
import 'dotenv/config';
import { Pool } from 'pg';

const pool = new Pool({ 
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  }
});

async function verifyTable() {
  try {
    // Check table structure
    const columns = await pool.query(`
      SELECT column_name, data_type 
      FROM information_schema.columns 
      WHERE table_name = 'users_background'
      ORDER BY ordinal_position;
    `);
    
    console.log('Table structure:', columns.rows);
    
    // Check sample data
    const data = await pool.query('SELECT * FROM users_background LIMIT 5;');
    console.log('Sample data:', data.rows);
    
    await pool.end();
  } catch (error) {
    console.error('Error verifying table:', error);
    await pool.end();
  }
}

verifyTable();