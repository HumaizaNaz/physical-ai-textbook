// drizzle.config.ts
import type { Config } from "drizzle-kit";
import "dotenv/config";

export default {
  schema: "./src/schema.ts", // Only our custom schema (users_background table)
  out: "./drizzle/migrations",
  dialect: "postgresql",
  dbCredentials: {
    url: process.env.DATABASE_URL!,
  },
  verbose: true,
  migrations: {
    schema: "./drizzle/migrations", // Location for migration files
  },
} satisfies Config;
