CREATE FUNCTION mark_bonus_cone_visited(
    cone_id BIGINT,
    visited_time TIMESTAMPTZ
) RETURNS TABLE (updated_id BIGINT) AS $$
BEGIN

    UPDATE bonus_cone AS b
    SET s.visited_time = visited_time
    WHERE b.id = cone_id;
    
    RETURN QUERY
    SELECT cone_id AS updated_id;
    
END;
$$ LANGUAGE PLPGSQL;