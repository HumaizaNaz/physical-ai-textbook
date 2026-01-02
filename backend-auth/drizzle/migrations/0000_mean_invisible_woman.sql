CREATE TABLE "users_background" (
	"user_id" uuid PRIMARY KEY NOT NULL,
	"programming_experience" text DEFAULT 'None' NOT NULL,
	"ros_experience" text DEFAULT 'None' NOT NULL,
	"linux_familiarity" text DEFAULT 'Beginner' NOT NULL,
	"hardware_experience" text DEFAULT 'None' NOT NULL,
	"electronics_knowledge" text DEFAULT 'None' NOT NULL,
	"robotics_projects" text DEFAULT 'None' NOT NULL,
	"learning_goal" text DEFAULT 'General Learning' NOT NULL
);
