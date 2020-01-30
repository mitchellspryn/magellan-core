CREATE FUNCTION create_simulation_run(
    _start_time TIMESTAMPTZ,
    _server_config JSONB,
    _client_config JSONB,
    _concrete_config JSONB,
    _simulation_id TEXT,
    _client_id TEXT,
    _bonus_cones bonus_cone_insert_type[],
    _spawn_pose spawn_pose_insert_type,
    _goal_pose goal_pose_insert_type
) RETURNS TABLE (inserted_id BIGINT) AS $$
DECLARE
    _running_status_id BIGINT;
    _inserted_simulation_run_id BIGINT;
BEGIN
    SELECT MAX(id) 
        INTO _running_status_id 
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
            _start_time,
            NULL,
            _running_status_id,
            _server_config,
            _client_config,
            _concrete_config,
            _simulation_id,
            _client_id, 
            NULL
        ) RETURNING id
    ) SELECT MAX(id)
        INTO _inserted_simulation_run_id
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
        _inserted_simulation_run_id,
        (_spawn_pose).location,
        (_spawn_pose).orientation
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
        _inserted_simulation_run_id,
        FALSE::boolean,
        'infinity'::real,
        (_goal_pose).cone_type,
        (_goal_pose).location,
        (_goal_pose).position_tolerance,
        (_goal_pose).velocity_tolerance
    );
    
    WITH packed AS
    (
        SELECT _bonus_cones AS r
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
            (cones).cone_id,
            _inserted_simulation_run_id,
            (cones).bonus_multiplier,
            NULL,
            (cones).cone_type,
            (cones).location
    FROM unnested;
    
    RETURN QUERY
    SELECT _inserted_simulation_run_id AS inserted_id;
    
END;
$$ LANGUAGE PLPGSQL;