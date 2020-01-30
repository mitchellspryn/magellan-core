CREATE FUNCTION mark_bonus_cone_visited(
    _run_id BIGINT,
    _cone_id BIGINT,
    _visited_time TIMESTAMPTZ
) RETURNS TABLE (updated_id BIGINT) AS $$
BEGIN

    UPDATE bonus_cone
    SET visited_time = _visited_time
    WHERE cone_id = _cone_id
        AND run_id = _run_id;
    
    RETURN QUERY
    SELECT _cone_id AS updated_id;
    
END;
$$ LANGUAGE PLPGSQL;