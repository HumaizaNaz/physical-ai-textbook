alter table "session" add index "session_userId_idx";

alter table "session" add column "userId" text not null references "user" ("id") on delete cascade;

alter table "account" add column "accountId" text not null;

alter table "account" add column "providerId" text not null;

alter table "account" add index "account_userId_idx";

alter table "account" add column "userId" text not null references "user" ("id") on delete cascade;

alter table "account" add column "accessToken" text;

alter table "account" add column "refreshToken" text;

alter table "account" add column "idToken" text;

alter table "account" add column "accessTokenExpiresAt" timestamptz;

alter table "account" add column "refreshTokenExpiresAt" timestamptz;

alter table "account" add column "createdAt" timestamptz default CURRENT_TIMESTAMP not null;

alter table "account" add column "updatedAt" timestamptz not null;

alter table "verification" add column "expiresAt" timestamptz not null;

alter table "verification" add column "createdAt" timestamptz default CURRENT_TIMESTAMP not null;

alter table "verification" add column "updatedAt" timestamptz default CURRENT_TIMESTAMP not null;