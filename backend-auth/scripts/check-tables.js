// Check all tables in the database
import 'dotenv/config';
import { Pool } from 'pg';

const pool = new Pool({ 
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  }
});

async function checkTables() {
  try {
    const result = await pool.query(`
      SELECT table_name 
      FROM information_schema.tables 
      WHERE table_schema = 'public'
      ORDER BY table_name;
    `);
    
    console.log('Tables in database:', result.rows);
    
    // Check specifically for better auth tables
    const authTables = result.rows.filter(row => 
      row.table_name.includes('auth') || 
      row.table_name.includes('user') ||
      row.table_name.includes('account') ||
      row.table_name.includes('session')
    );
    
    console.log('Potential auth-related tables:', authTables);
    
    await pool.end();
  } catch (error) {
    console.error('Error checking tables:', error);
    await pool.end();
  }
}

checkTables();