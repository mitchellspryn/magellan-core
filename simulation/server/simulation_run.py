import threading
import time
import numpy as np
import requests
import json
import io
import tzlocal
import pause
import datetime
import queue
import traceback

import airsim
import airsim.airsim_types as at

import rm_bot_client.rm_bot_client as rm_bot_client
import robo_magellan_orchestrator.robo_magellan_orchestrator as robo_magellan_orchestrator
import simulation_database.simulation_database_manager as simulation_database_manager

import simulation_instance_manager

class SimulationRun(object):
    def __init__(self, client_config, simulation_instance_manager, database_client, server_config):
        self.is_running = True

        self.client = rm_bot_client.RmBotClient()
        self.orchestrator = robo_magellan_orchestrator.RoboMagellanCompetitionOrchestrator(client_config)
        self.database_client = database_client
        self.simulation_instance_manager = simulation_instance_manager
        self.exception_queue = queue.Queue()

        # TODO: For now, make client config == concrete config
        self.depth_image_callback_url = client_config['depth_image_callback_url']
        self.lidar_callback_url = client_config['lidar_callback_url']
        self.ground_camera_callback_url = client_config['ground_callback_url']
        self.imu_gps_callback_url = client_config['imu_gps_callback_url']
        self.run_complete_callback_url = client_config['run_complete_callback_url']

        self.background_threads = []
        if (self.depth_image_callback_url is not None):
            self.background_threads.append(threading.Thread(target=self.__publish_depth_image_bg_worker, args=()))
        if (self.lidar_callback_url is not None):
            self.background_threads.append(threading.Thread(target=self.__publish_lidar_bg_worker, args=()))
        if (self.ground_camera_callback_url is not None):
            self.background_threads.append(threading.Thread(target=self.__publish_ground_image_bg_worker, args=()))
        if (self.imu_gps_callback_url is not None):
            self.background_threads.append(threading.Thread(target=self.__publish_imu_gps_bg_worker, args=()))

        self.background_threads.append(threading.Thread(target=self.__bot_location_monitor_bg_worker, args=()))
        self.monitor_thread = threading.Thread(target=self.__main_monitor_thread, args=())

        self.timezone = tzlocal.get_localzone()
        self.is_running = True
        self.error = None
        self.error_stack_trace = None
        self.client_id = None

        if 'client_id' in client_config:
            self.client_id = client_config[client_id]

        if 'debug_draw' in client_config and client_config['debug_draw']:
            self.orchestrator.set_debug_draw_enabled(True)
        else:
            self.orchestrator.set_debug_draw_enabled(False)

        self.random_seed = 42
        if 'random_seed' in client_config:
            self.random_seed = client_config['random_seed']

        self.orchestrator.set_random_seed(client_config['random_seed'])

        self.start_time = datetime.datetime.now(tz=self.timezone)
        self.end_time = None

        self.run_id = self.database_client.create_simulation_run_function(self.start_time,
                                                                          self.server_config,
                                                                          client_config,
                                                                          client_config, # for now, concrete and logical config are the same thing
                                                                          self.client_id,
                                                                          self.simulation_instance_manager.simulation_execution_id,
                                                                          self.orchestrator.cones,
                                                                          self.orchestrator.start_pose,
                                                                          self.orchestrator.goal_pose)


        self.monitor_thread.start()

        for callback in self.background_threads:
            callback.start()

    def set_control_signals(self, left_throttle, right_throttle):
        if (self.is_running):
            self.client.drive(left_throttle, right_throttle)

    def cancel(self):
        self.exception_queue.put(('The operation was cancelled by the user.', None))

    def __main_monitor_thread(self):
        try:
            now = datetime.datetime.now(tzinfo=self.timezone)
            self.orchestrator.start_new_run(self.client)
            status = self.orchestrator.get_run_summary(self.client)
            visited_cone_ids = set()
            while(not status['runComplete']):
                if not self.exception_queue.empty():
                    val = self.exception_queue.get()
                    self.error = val[0]
                    self.error_stack_trace = val[1]
                    break

                for cone in self.orchestrator.cones:
                    if cone.visited and cone.cone_id not in visited_cone_ids:
                        self.database_client.mark_bonus_cone_visited(cone, self.run_id)
                        visited_cone_ids.add(cone.cone_id)

                if (not self.simulation_instance_manager.is_simulation_alive()):
                    raise RuntimeError('The simulator crashed. Check {0} for logs'.format(self.simulation_instance_manager.get_log_path()))

                self.orchestrator.run_tick(self.client)
                status = self.orchestrator.get_run_summary(self.client)
        except Exception as e:
            self.error = str(e)
            self.error_stack_trace = traceback.format_exc()

        self.is_running = False

        # Attempt to stop the car.
        # It's no big deal if it doesn't stop.
        try:
            self.client.drive(0, 0)
        except:
            pass

        self.end_time = datetime.datetime.now(tzinfo=self.timezone)
        self.database_client.complete_simulated_run(self.run_id,
                                                    self.end_time,
                                                    self.error,
                                                    self.error_stack_trace,
                                                    self.orchestrator.goal_point.closest_distance,
                                                    self.orchestrator.goal_point.visited)

        for background_thread in self.background_threads:
            background_thread.join()

        if (self.run_complete_callback_url is not None):
            post_data = {}
            post_data['end_time'] = self.end_time
            post_data['error'] = self.error
            post_data['error_stack_trace'] = self.error_stack_trace
            post_data['goal_visited'] = self.orchestrator.goal_point.visited

            requests.post(self.run_complete_callback_url, json=post_data)

    def __bot_location_monitor_bg_worker(self):
        try:
            now = datetime.datetime.now(tzinfo=self.timezone)
            while(self.is_running):
                next_now = now + datetime.timedelta(seconds = 1.0 / 2.0) # 2 hz
                response = self.client.simGetVehiclePose()

                self.database_client.insert_bot_pose(self.run_id,
                                                     now,
                                                     response.position.x_val,
                                                     response.position.y_val,
                                                     response.position.z_val,
                                                     reponse.orientation.w_val,
                                                     response.orientation.x_val,
                                                     response.orientation.y_val,
                                                     response.orientation.z_val)

                now = next_now
                pause.until(now)
        except Exception as e:
            self.exception_queue.put((str(e), traceback.format_exc()))
            raise

    def __publish_depth_image_bg_worker(self):
        try:
            local_data = None
            now = datetime.datetime.now(tz=self.timezone)
            while (self.is_running):
                now += datetime.timedelta(seconds = 1.0 / 30.0) # 30 FPS

                response = self.client.simGetImages([
                    at.ImageRequest('Xtion', at.ImageType.Scene, pixels_as_float=False, compress=False),
                    at.ImageRequest('Xtion', at.ImageType.DepthPlanner, pixels_as_float=True, compress=False)]) # Misspelled "Planar"

                if (local_data is None):
                    local_data = np.zeros((response[0].shape[0], response[0].shape[1], 4), dtype=np.float32)

                local_data[:, :, 0:3] = response[0]
                local_data[:, :, 3] = response[1]

                post_data = {}
                post_data['shape'] = local_data.shape
                post_data['data'] = local_data.tostring(order='C')

                response = requests.post(self.depth_image_callback_url, json=post_data)

                pause.until(now)
        except Exception as e:
            self.exception_queue.put((str(e), traceback.format_exc()))
            raise

    def __publish_lidar_bg_worker(self):
        try:
            now = datetime.datetime.now(tz=self.timezone)
            while (self.is_running):
                now += datetime.timedelta(seconds= 1.0 / 5.0) # 5 Hz

                response = self.client.getLidarData('Lidar')

                post_data = {}
                post_data['shape'] = (response.point_cloud.shape / 3, 3)

                response = requests.post(self.lidar_callback_url, json=post_data)

                pause.until(now)
        except Exception as e:
            self.exception_queue.put((str(e), traceback.format_exc()))
            raise


    def __publish_ground_image_bg_worker(self):
        try:
            now = datetime.datetime.now(tz=self.timezone)
            while (self.is_running):
                now += datetime.timedelta(seconds = 1.0 / 30.0) # 30 FPS

                response = self.client.simGetImages([
                    at.ImageRequest('Ground', at.ImageType.Scene, pixels_as_float=False, compress=False)])

                post_data = {}
                post_data['shape'] = response[0].shape
                post_data['data'] = response[0].tostring(order='C')

                response = requests.post(self.ground_camera_callback_url, json=post_data)

                pause.until(now)
        except Exception as e:
            self.exception_queue.put((str(e), traceback.format_exc()))
            raise

    def __publish_imu_gps_bg_worker(self):
        try:
            now = datetime.datetime.now(tzlocal=self.timezone)

            while(self.is_running):
                now += datetime.timedelta(seconds = 1.0 / 20.0) # 20 hz

                response = self.client.readSensors()

                post_data = {}
                post_data['imu_x'] = response['readings']['BodyImu']['IMU-Lin-x']
                post_data['imu_y'] = response['readings']['BodyImu']['IMU-Lin-y']
                post_data['imu_z'] = response['readings']['BodyImu']['IMU-Lin-z']
                post_data['imu_roll'] = response['readings']['BodyImu']['IMU-Ang-roll']
                post_data['imu_pitch'] = response['readings']['BodyImu']['IMU-Ang-pitch']
                post_data['imu_yaw'] = response['readings']['BodyImu']['IMU-Ang-yaw']

                post_data['latitude'] = response['readings']['GPS']['geo_point']['latitude']
                post_data['longitude'] = response['readings']['GPS']['geo_point']['longitude']
                post_data['altitude'] = response['readings']['GPS']['geo_point']['altitude']

                post_data['mag_x'] = response['readings']['Magentometer']['Mag-Vec-x']
                post_data['mag_y'] = response['readings']['Magentometer']['Mag-Vec-y']
                post_data['mag_z'] = response['readings']['Magentometer']['Mag-Vec-z']

                response = requests.post(self.imu_gps_callback_url, json=post_data)

                pause.until(now)
        except Exception as e:
            self.exception_queue.put((str(e), traceback.format_exc()))
            raise



