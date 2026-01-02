// api/_root.ts - Root handler for Vercel
export default function handler(req: any, res: any) {
  res.status(200).json({ 
    message: 'Backend Auth API is running!',
    endpoints: {
      'GET /api/test': 'Test endpoint',
      'GET /api/health': 'Health check',
      'POST /api/auth/sign-up/email': 'Sign up',
      'POST /api/auth/sign-in/email': 'Sign in',
      'GET /api/user/me': 'Get user profile',
      'POST /api/user/background': 'Update user profile'
    }
  });
}