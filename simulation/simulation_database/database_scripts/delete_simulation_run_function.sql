CREATE FUNCTION delete_simulation_run(
    run_id_to_delete BIGINT
) RETURNS TABLE (deleted_run_id BIGINT) AS $$
BEGIN
    DELETE 
    FROM simulation_run
    WHERE id = run_id_to_delete;
    
    DELETE FROM bonus_cone
    WHERE run_id = run_id_to_delete;
    
    DELETE FROM spawn_pose
    WHERE run_id = run_id_to_delete;
    
    DELETE FROM goal_pose
    WHERE run_id = run_id_to_delete;
    
END;
$$ LANGUAGE PLPGSQL;