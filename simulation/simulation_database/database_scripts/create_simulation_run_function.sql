CREATE FUNCTION create_simulation_run(
    start_time TIMESTAMPTZ,
    server_config JSONB,
    client_config JSONB,
    concrete_config JSONB,
    simulation_id TEXT,
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
            server_config,
            client_config,
            concrete_config,
            simulation_id,
            client_id,
            error
        )
        VALUES
        (
            DEFAULT,
            start_time,
            NULL,
            inserted_simulation_run_id,
            server_config,
            client_config,
            concrete_config,
            simulation_id,
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
        location,
        orientation
    )
    VALUES
    (
        DEFAULT,
        inserted_simulation_run_id,
        (spawn_pose).location,
        (spawn_pose).orientation
    );
    
    INSERT INTO goal_pose 
    (
        id,
        run_id,
        goal_reached,
        bot_closest_distance,
        cone_type,
        location,
        position_tolerance,
        velocity_tolerance
    )
    VALUES
    (
        DEFAULT,
        inserted_simulation_run_id,
        FALSE,
        'infinity'::real,
        (goal_pose).cone_type,
        (goal_pose).location,
        (goal_pose).position_tolerance,
        (goal_pose).velocity_tolerance
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
        cone_id,
        run_id,
        bonus_multiplier,
        visited_time,
        cone_type,
        location
    )
    SELECT  --DEFAULT,
            inserted_simulation_run_id,
            cone_id,
            (cones).bonus_multiplier,
            NULL,
            (cones).cone_type,
            (cones).location
    FROM unnested;
    
    RETURN QUERY
    SELECT inserted_simulation_run_id AS inserted_id;
    
END;
$$ LANGUAGE PLPGSQL;