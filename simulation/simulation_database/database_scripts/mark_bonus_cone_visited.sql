CREATE FUNCTION mark_bonus_cone_visited(
    run_id BIGINT,
    cone_id BIGINT,
    visited_time TIMESTAMPTZ
) RETURNS TABLE (updated_id BIGINT) AS $$
BEGIN

    UPDATE bonus_cone AS b
    SET s.visited_time = visited_time
    WHERE b.cone_id = cone_id
        AND b.run_id = run_id
    
    RETURN QUERY
    SELECT cone_id AS updated_id;
    
END;
$$ LANGUAGE PLPGSQL;