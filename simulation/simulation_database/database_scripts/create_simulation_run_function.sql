CREATE FUNCTION create_simulation_run(
    start_time TIMESTAMPTZ,
    config JSONB,
    concrete_config JSONB,
    client_id TEXT,
    bonus_cones bonus_cone_insert_type[],
    spawn_pose spawn_pose_insert_type,
    goal_pose goal_pose_insert_type
) RETURNS TABLE (inserted_id BIGINT) AS $$
DECLARE
    running_status_id BIGINT;
    inserted_simulation_run_id BIGINT;
BEGIN
    SELECT MAX(id) 
        INTO inserted_simulation_run_id 
    FROM status
    WHERE description = 'Running';

    WITH insert_sim_run AS
    (
        INSERT INTO simulation_run 
        (
            id,
            start_time,
            end_time,
            status,
            config,
            concrete_config,
            client_id,
            error
        )
        VALUES
        (
            DEFAULT,
            start_time,
            NULL,
            inserted_simulation_run_id,
            config,
            concrete_config,
            client_id, 
            NULL
        ) RETURNING id
    ) SELECT MAX(id)
        INTO inserted_simulation_run_id
    FROM insert_sim_run;
    
    INSERT INTO spawn_pose
    (
        id,
        run_id,
        start_x,
        start_y,
        start_z,
        start_roll,
        start_pitch,
        start_yaw
    )
    VALUES
    (
        DEFAULT,
        inserted_simulation_run_id,
        (spawn_pose).start_x,
        (spawn_pose).start_y,
        (spawn_pose).start_z,
        (spawn_pose).start_roll,
        (spawn_pose).start_pitch,
        (spawn_pose).start_yaw
    );
    
    INSERT INTO goal_pose 
    (
        id,
        run_id,
        visited_time,
        bot_closest_distance,
        cone_type,
        goal_x,
        goal_y,
        goal_z
    )
    VALUES
    (
        DEFAULT,
        inserted_simulation_run_id,
        FALSE,
        'infinity'::real,
        (goal_pose).cone_type,
        (goal_pose).goal_x,
        (goal_pose).goal_y,
        (goal_pose).goal_z
    );
    
    WITH packed AS
    (
        SELECT bonus_cones AS r
    ),
    unnested AS
    (
        SELECT UNNEST(r) as cones
        FROM packed
    )
    INSERT INTO bonus_cone
    (
        run_id,
        bonus_multiplier,
        visited_time,
        cone_type,
        cone_spawn_x,
        cone_spawn_y,
        cone_spawn_z
    )
    SELECT  --DEFAULT,
            inserted_simulation_run_id,
            (cones).bonus_multiplier,
            NULL,
            (cones).cone_type,
            (cones).cone_spawn_x,
            (cones).cone_spawn_y,
            (cones).cone_spawn_z
    FROM unnested;
    
    RETURN QUERY
    SELECT inserted_simulation_run_id AS inserted_id;
    
END;
$$ LANGUAGE PLPGSQL;