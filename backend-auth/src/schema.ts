import { pgTable, text, uuid } from "drizzle-orm/pg-core";

// User background information (links to Better-Auth's user table via user_id)
export const users_background = pgTable("users_background", {
  user_id: uuid("user_id").primaryKey().notNull(), // Foreign key to reference Better-Auth's user table
  programming_experience: text("programming_experience").notNull().default('None'),
  ros_experience: text("ros_experience").notNull().default('None'),
  linux_familiarity: text("linux_familiarity").notNull().default('Beginner'),
  hardware_experience: text("hardware_experience").notNull().default('None'),
  electronics_knowledge: text("electronics_knowledge").notNull().default('None'),
  robotics_projects: text("robotics_projects").notNull().default('None'),
  learning_goal: text("learning_goal").notNull().default('General Learning'),
});