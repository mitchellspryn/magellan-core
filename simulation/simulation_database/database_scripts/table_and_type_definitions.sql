CREATE TYPE Vector3r AS (
    x REAL,
    y REAL,
    z REAL
);

CREATE TYPE Quaternionr AS (
    w REAL,
    x REAL,
    y REAL,
    z REAL
);

CREATE TABLE simulation_run (
    id BIGSERIAL PRIMARY KEY NOT NULL,
    start_time TIMESTAMPTZ NOT NULL, 
    end_time TIMESTAMPTZ NULL,
    status INT NOT NULL,
    server_config JSONB NOT NULL,
    client_config JSONB NOT NULL,
    concrete_config JSONB NOT NULL,
    simulation_id TEXT NOT NULL,
    client_id TEXT NULL,
    error TEXT NULL,
    error_stack_trace TEXT NULL
);

CREATE TYPE simulation_run_insert_type AS (
    start_time TIMESTAMPTZ, 
    end_time TIMESTAMPTZ,
    status INT,
    config JSONB,
    concrete_config JSONB,
    client_id TEXT,
    error TEXT,
    error_stack_trace TEXT
);

CREATE TABLE bonus_cone (
    id BIGSERIAL PRIMARY KEY NOT NULL,
    run_id BIGINT NOT NULL,
    cone_id INT NOT NULL,
    bonus_multiplier REAL NOT NULL,
    visited_time TIMESTAMPTZ NULL,
    cone_type TEXT NOT NULL,
    location Vector3r NOT NULL
);

CREATE INDEX idx_bonus_cone_run_id ON bonus_cone(run_id);
CREATE INDEX idx_bonus_cone_cone_id_run_id ON bonus_cone(cone_id, run_id);

CREATE TYPE bonus_cone_insert_type AS (
    cone_id INT,
    bonus_multiplier REAL,
    cone_type TEXT,
    location Vector3r
);

CREATE TABLE spawn_pose (
    id BIGSERIAL PRIMARY KEY NOT NULL,
    run_id BIGINT NOT NULL,
    location Vector3r NOT NULL,
    orientation Quaternionr NOT NULL
);

CREATE INDEX idx_spawn_pose_run_id ON spawn_pose(run_id);

CREATE TYPE spawn_pose_insert_type AS (
    location Vector3r,
    orientation Quaternionr
);

CREATE TABLE goal_pose (
    id BIGSERIAL PRIMARY KEY NOT NULL,
    run_id BIGINT NOT NULL,
    goal_reached BOOLEAN NOT NULL,
    bot_closest_distance REAL NULL,
    cone_type TEXT NULL,
    location Vector3r NOT NULL,
    position_tolerance REAL NOT NULL,
    velocity_tolerance REAL NOT NULL
);

CREATE INDEX idx_goal_pose_run_id ON goal_pose(run_id);

CREATE TYPE goal_pose_insert_type AS (
    cone_type TEXT,
    location Vector3r,
    position_tolerance REAL,
    velocity_tolerance REAL
);

CREATE TABLE status (
    id BIGINT NOT NULL,
    description TEXT NOT NULL
);

CREATE INDEX idx_status_id ON status(id);

CREATE TABLE bot_pose (
    id BIGSERIAL PRIMARY KEY NOT NULL,
    run_id BIGINT NOT NULL,
    timestamp TIMESTAMPTZ NOT NULL,
    location Vector3r NOT NULL,
    orientation Quaternionr NOT NULL
);

CREATE INDEX idx_bot_pose_run_id ON bot_pose(run_id);

ALTER TABLE bonus_cone CLUSTER ON idx_bonus_cone_cone_id_run_id;
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

CREATE VIEW simulation_run_summary AS
    SELECT  s.id,
            s.start_time,
            s.end_time,
            st.description,
            s.simulation_id,
            s.client_id,
            s.error
    FROM simulation_run AS s
    INNER JOIN status AS st
        ON s.status = st.id;
