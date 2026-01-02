// drizzle.config.better-auth.ts
import type { Config } from "drizzle-kit";
import "dotenv/config";

export default {
  schema: "./auth-schema.ts", // Only point to the better-auth generated schema
  out: "./drizzle/better-auth-migrations", // Use a separate folder for these migrations
  dialect: "postgresql",
  dbCredentials: {
    url: process.env.DATABASE_URL!,
  },
  verbose: true,
} satisfies Config;
