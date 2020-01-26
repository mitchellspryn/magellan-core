CREATE FUNCTION multiply(REAL, REAL) RETURNS REAL AS
$$
    SELECT $1 * $2;
$$
LANGUAGE SQL
IMMUTABLE STRICT;

CREATE AGGREGATE multiply_aggregate(basetype=REAL, sfunc=multiply, stype=REAL, initcond=1);

CREATE FUNCTION compute_run_score(run_id_param BIGINT) RETURNS REAL 
AS $$
    WITH cone_multiplier AS
    (
        SELECT multiply_aggregate(bonus_multiplier) AS multiplier
        FROM bonus_cone AS b
        WHERE b.run_id = run_id_param
        AND b.visited_time IS NOT NULL
    )
    SELECT CASE st.Description WHEN 'Not Started' THEN 0
                               WHEN 'Running' THEN 0
                               WHEN 'Completed Success' THEN
                                CASE WHEN gp.goal_reached THEN
                                    (EXTRACT(EPOCH FROM (sr.end_time - sr.start_time)) * c.multiplier)::REAL
                                ELSE
                                    0 --TODO: What should we use for scoring here?
                                END
                               WHEN 'Completed Failure' THEN 0
                               ELSE -1 END
    FROM status AS st
    INNER JOIN simulation_run AS sr
    ON sr.status = st.id
    INNER JOIN goal_pose AS gp
    ON gp.run_id = sr.id
    CROSS JOIN cone_multiplier AS c --one row
    WHERE sr.id = run_id_param;
$$
LANGUAGE SQL
IMMUTABLE
RETURNS NULL ON NULL INPUT;
