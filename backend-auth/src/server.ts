// backend-auth/src/server.ts
// This file is for local development only
// For Vercel deployment, the api/index.ts file is used

import 'dotenv/config';
import { serve } from '@hono/node-server';
import { Hono } from 'hono';
import { cors } from 'hono/cors';

const app = new Hono();
const PORT = process.env.PORT ? parseInt(process.env.PORT, 10) : 7860;
const FRONTEND_URL = process.env.FRONTEND_URL || 'http://localhost:3000';

// --- Middleware ---
app.use('*', cors({ origin: FRONTEND_URL, credentials: true }));

// --- Test Route ---
app.get('/', (c) => {
  return c.text('Local development server is running! For Vercel deployment, use the api/index.ts file.');
});

app.get('/test', (c) => {
  return c.text('Local test server is running!');
});

console.log(`Local development server running on http://localhost:${PORT}`);

serve({
  fetch: app.fetch,
  port: PORT,
});