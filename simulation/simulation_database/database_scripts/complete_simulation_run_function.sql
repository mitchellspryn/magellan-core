CREATE FUNCTION complete_simulated_run(
    run_id BIGINT,
    run_end_time TIMESTAMPTZ,
    error TEXT,
    bot_closest_distance REAL,
    goal_reached BOOLEAN
) RETURNS TABLE (updated_id BIGINT) AS $$
BEGIN

    UPDATE simulation_run AS s
    SET s.end_time = end_time,
        s.error = error
    WHERE s.id = run_id;
    
    UPDATE goal_pose as g
    SET g.goal_reached = goal_reached,
        g.bot_closest_distance = bot_closest_distance
    WHERE g.run_id = run_id;
    
    RETURN QUERY
    SELECT run_id AS updated_id;
    
END;
$$ LANGUAGE PLPGSQL;