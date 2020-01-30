import datetime
import json
import os
import psycopg2
import psycopg2.pool
import pytz


class SimulationDatabaseManager(object):
    """description of class"""
    def __init__(self, db_user, db_password, db_host, db_port, db_database):
        self.connection_pool = psycopg2.pool.SimpleConnectionPool(1, 
                                                                  20, 
                                                                  user = db_user,
                                                                  password = db_password,
                                                                  host = db_host,
                                                                  port = db_port, 
                                                                  database = db_database)

        if not self.connection_pool:
            raise ValueError('Unable to initialize connection pool to database {0} at {1}:{2} using user {3}, password {4}'
                             .format(db_database, db_host, db_posrt, db_user, db_password))

    def __del__(self):
        if self.connection_pool is not None:
            self.connection_pool.closeall()
            self.connection_pool = None

    def create_simulation_run_function(self, 
                                        run_start_time,
                                        server_config,
                                        client_config,
                                        concrete_run_config,
                                        simulation_id,
                                        client_id,
                                        bonus_cones,
                                        spawn_pose,
                                        goal_pose):
        
        parameters = {}
        parameters['start_time'] = run_start_time
        parameters['server_config'] = json.dumps(server_config)
        parameters['client_config'] = json.dumps(client_config)
        parameters['concrete_config'] = json.dumps(concrete_run_config)
        parameters['client_id'] = client_id
        parameters['simulation_id'] = simulation_id
        parameters['bonus_cones'] = [bc.to_db_tuple() for bc in bonus_cones]
        parameters['spawn_pose'] = spawn_pose.to_db_tuple()
        parameters['goal_pose'] = goal_pose.to_db_tuple()

        query = """
            SELECT create_simulation_run(
                %(start_time)s::TIMESTAMPTZ, 
                %(server_config)s::JSONB,
                %(client_config)s::JSONB,
                %(concrete_config)s::JSONB,
                %(client_id)s::TEXT,
                %(simulation_id)s::TEXT,
                %(bonus_cones)s::bonus_cone_insert_type[],
                %(spawn_pose)s::spawn_pose_insert_type,
                %(goal_pose)s::goal_pose_insert_type);
        """

        result = self.__execute(query, parameters)
        run_id = result[0][0]
        
        return run_id

    def mark_bonus_cone_visited(self, cone, run_id):
        parameters = {}
        parameters['run_id'] = run_id 
        parameters['cone_id'] = cone.cone_id
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
                               error_stack_trace,
                               goal_pose):
        parameters = {}
        parameters['run_id'] = run_id
        parameters['run_end_time'] = run_end_time
        parameters['error'] = error
        parameters['error_stack_trace'] = error_stack_trace
        parameters['bot_closest_distance'] = goal_pose.closest_distance
        parameters['goal_reached'] = goal_pose.visited

        query = """
            SELECT complete_simulated_run(
                %(run_id)s::BIGINT,
                %(run_end_time)s::TIMESTAMPTZ,
                %(error)s::TEXT,
                %(error_stack_trace)s::TEXT,
                %(bot_closest_distance)s::REAL,
                %(goal_reached)s::BOOLEAN
            );
        """

        self.__execute(query, parameters, nonquery=True)

    def insert_bot_pose(self, 
                        run_id,
                        timestamp,
                        position,
                        orientation):
        parameters = {}
        parameters['run_id'] = run_id
        parameters['timestamp'] = timestamp
        parameters['location'] = (position.x_val, position.y_val, position.z_val)
        parameters['orientation'] = (orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val)

        query = """
            INSERT INTO bot_pose
            (
                run_id,
                timstamp,
                location,
                orientation
            )
            VALUES
            (
                DEFAULT,
                %(run_id)s::BIGINT,
                %(timestamp)s::TIMESTAMPTZ,
                %(location)s::Vector3r,
                %(orientation)s::Quaternionr
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


