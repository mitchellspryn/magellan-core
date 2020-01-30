CREATE FUNCTION complete_simulated_run(
    _run_id BIGINT,
    _run_end_time TIMESTAMPTZ,
    _error TEXT,
    _error_stack_trace TEXT,
    _bot_closest_distance REAL,
    _goal_reached BOOLEAN
) RETURNS TABLE (updated_id BIGINT) AS $$
DECLARE 
    _filter_string TEXT;
    _status_id BIGINT;
BEGIN
    SELECT CASE WHEN _error IS NULL THEN 'Completed Success' ELSE 'Completed Failure' END 
        INTO _filter_string;

    SELECT MAX(id)
        INTO _status_id
    FROM status
    WHERE description = _filter_string;

    UPDATE simulation_run
    SET end_time = _run_end_time,
        error = _error,
        error_stack_trace = _error_stack_trace,
        status = _status_id
    WHERE id = _run_id;
    
    UPDATE goal_pose
    SET goal_reached = _goal_reached,
        bot_closest_distance = _bot_closest_distance
    WHERE run_id = _run_id;
    
    RETURN QUERY
    SELECT _run_id AS updated_id;
    
END;
$$ LANGUAGE PLPGSQL;