import datetime
import json
import os
import psycopg2
import pytz


class SimulationDatabaseManager(object):
    """description of class"""
    def __init__(self):
        self.connection_pool = psycopg2.pool.SimpleConnectionPool(1, 
                                                                  20, 
                                                                  user = 'postgres',
                                                                  password = 'abcd',
                                                                  host = '127.0.0.1',
                                                                  port = '5432', 
                                                                  database = 'simulation')

    def __del__(self):
        if self.connection_pool is not None:
            self.connection_pool.closeall()
            self.connection_pool = None

    def create_simulation_run_function(self, 
                                        run_start_time,
                                        run_config,
                                        concrete_run_config,
                                        client_id,
                                        bonus_cones,
                                        spawn_pose,
                                        goal_pose):
        
        parameters = {}
        parameters['start_time'] = run_start_time
        parameters['config'] = json.dumps(run_config)
        parameters['concrete_config'] = json.dumps(concrete_run_config)
        parameters['client_id'] = client_id
        parameters['bonus_cones'] = [bc.to_db_tuple() for bc in bonus_cones]
        parameters['spawn_pose'] = spawn_pose.to_db_tuple()
        parameters['goal_pose'] = goal_pose.to_db_tuple()

        query = """
            SELECT create_simulation_run(
                %(start_time)s::TIMESTAMPTZ, 
                %(config)s::JSONB,
                %(concrete_config)s::JSONB,
                %(client_id)s::TEXT,
                %(bonus_cones)s::bonus_cone_insert_type[],
                %(spawn_pose)s::spawn_pose_insert_type,
                %(goal_pose)s::goal_pose_insert_type);
        """

        result = self.__execute(query, parameters)
        run_id = result[0][0]
        
        return run_id

    def get_bonus_cones_for_run(self, run_id):
        parameters = {}
        parameters['run_id'] = run_id

        query = """
            SELECT cone_spawn_x,
                   cone_spawn_y,
                   cone_spawn_z,
                   cone_id
            FROM bonus_cone
            WHERE run_id = %s(run_id);
        """

        results = self.__execute(query, parameters)
        
        mapping = {}
        for result in results:
            key = (result[0], result[1], result[2])
            mapping[key] = result[3]

    def mark_bonus_cone_visited(self, cone):
        parameters = {}
        
        parameters['cone_id'] = cone.id
        parameters['visited_time'] = cone.visited_time_stamp

        query = """
            SELECT mark_bonus_cone_visited(
                %(cone_id)s::BIGINT,
                %(visited_time)s::TIMESTAMPTZ
            );
        """

        self.__execute(query, parameters, nonquery=True)

    def complete_simulated_run(self, 
                               run_id,
                               run_end_time,
                               error,
                               goal_pose):
        parameters = {}
        parameters['run_id'] = run_id
        parameters['run_end_time'] = run_end_time
        parameters['error'] = error
        parameters['bot_closest_distance'] = goal_pose.closest_distance
        parameters['goal_reached'] = goal_pose.visited

        query = """
            SELECT complete_simulated_run(
                %(run_id)s::BIGINT,
                %(run_end_time)s::TIMESTAMPTZ,
                %(error)s::TEXT,
                %(bot_closest_distance)s::REAL,
                %(goal_reached)s::BOOLEAN
            );
        """

        self.__execute(query, parameters, nonquery=True)

    def __execute(self, query, parameters, nonquery = False):
        try:
            connection = self.connection_pool.getconn()

            cursor = connection.cursor()
            cursor.execute(query, parameters)

            result = None
            if (not nonquery):
                result = cursor.fetchall()

            connection.commit()
            cursor.close()

        finally:
            if (connection):
                self.connection_pool.putconn(connection)


