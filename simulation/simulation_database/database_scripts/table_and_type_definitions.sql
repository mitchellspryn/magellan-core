CREATE TABLE simulation_run (
    id BIGSERIAL PRIMARY KEY NOT NULL,
    start_time TIMESTAMPTZ NOT NULL, 
    end_time TIMESTAMPTZ NULL,
    status INT NOT NULL,
    config JSONB NOT NULL,
    concrete_config JSONB NOT NULL,
    client_id TEXT NULL,
    error TEXT NULL
);

CREATE TYPE simulation_run_insert_type AS (
    start_time TIMESTAMPTZ, 
    end_time TIMESTAMPTZ,
    status INT,
    config JSONB,
    concrete_config JSONB,
    client_id TEXT,
    error TEXT
);

CREATE TABLE bonus_cone (
    cone_id BIGSERIAL PRIMARY KEY NOT NULL,
    run_id BIGINT NOT NULL,
    bonus_multiplier REAL NOT NULL,
    visited_time TIMESTAMPTZ NULL,
    cone_type TEXT NOT NULL,
    cone_spawn_x REAL NOT NULL,
    cone_spawn_y REAL NOT NULL,
    cone_spawn_z REAL NOT NULL
);

CREATE INDEX idx_bonus_cone_run_id ON bonus_cone(run_id);

CREATE TYPE bonus_cone_insert_type AS (
    bonus_multiplier REAL,
    cone_type TEXT,
    cone_spawn_x REAL,
    cone_spawn_y REAL,
    cone_spawn_z REAL
);

CREATE TABLE spawn_pose (
    id BIGSERIAL PRIMARY KEY NOT NULL,
    run_id BIGINT NOT NULL,
    start_x REAL NOT NULL,
    start_y REAL NOT NULL,
    start_z REAL NOT NULL,
    start_roll REAL NOT NULL,
    start_pitch REAL NOT NULL,
    start_yaw REAL NOT NULL
);

CREATE INDEX idx_spawn_pose_run_id ON spawn_pose(run_id);

CREATE TYPE spawn_pose_insert_type AS (
    start_x REAL,
    start_y REAL,
    start_z REAL,
    start_roll REAL,
    start_pitch REAL,
    start_yaw REAL
);

CREATE TABLE goal_pose (
    id BIGSERIAL PRIMARY KEY NOT NULL,
    run_id BIGINT NOT NULL,
    goal_reached BOOLEAN NOT NULL,
    bot_closest_distance REAL NULL,
    cone_type TEXT NULL,
    goal_x REAL NOT NULL,
    goal_y REAL NOT NULL,
    goal_z REAL NOT NULL
);

CREATE INDEX idx_goal_pose_run_id ON goal_pose(run_id);

CREATE TYPE goal_pose_insert_type AS (
    cone_type TEXT,
    goal_x REAL,
    goal_y REAL,
    goal_z REAL
);

CREATE TABLE status (
    id BIGINT NOT NULL,
    description TEXT NOT NULL
);

CREATE INDEX idx_status_id ON status(id);

ALTER TABLE bonus_cone CLUSTER ON idx_bonus_cone_run_id;
ALTER TABLE spawn_pose CLUSTER ON idx_spawn_pose_run_id;
ALTER TABLE goal_pose CLUSTER ON idx_goal_pose_run_id;
ALTER TABLE status CLUSTER ON idx_status_id;

TRUNCATE TABLE status;

INSERT INTO status
VALUES 
(
    1,
    'Not Started'
),
(
    2,
    'Running'
),
(
    3,
    'Completed Success'
),
(
    4,
    'Completed Failure'
);