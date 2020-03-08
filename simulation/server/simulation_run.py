import threading
import time
import numpy as np
import requests
import json
import io
import tzlocal
import datetime
import queue
import traceback
import base64

import airsim
import airsim.airsim_types as at

import rm_bot_client.rm_bot_client as rm_bot_client
import robo_magellan_orchestrator.robo_magellan_orchestrator as robo_magellan_orchestrator
import simulation_database.simulation_database_manager as simulation_database_manager

import simulator_instance_manager

class SimulationRun(object):
    class LockedSection(object):
        def __init__(self, parent):
            self.parent = parent

        def __enter__(self):
            self.parent.client_lock.acquire(blocking=True)

        def __exit__(self, type, value, traceback):
            self.parent.client_lock.release()

    def __init__(self, client_config, simulation_instance_manager, database_client, server_config):
        self.is_running = True
        self.is_complete = False
        self.last_status = None
        self.server_config = server_config
        self.client_lock = threading.Lock()

        self.random_seed = 42
        if 'random_seed' in client_config:
            self.random_seed = client_config['random_seed']

        self.client = rm_bot_client.RmBotClient()
        self.orchestrator = robo_magellan_orchestrator.RoboMagellanCompetitionOrchestrator(client_config, self.client, self.random_seed)
        self.database_client = database_client
        self.simulation_instance_manager = simulation_instance_manager
        self.exception_queue = queue.Queue()

        # TODO: For now, make client config == concrete config
        self.depth_image_callback_url = client_config['depth_image_callback_url'] if 'depth_image_callback_url' in client_config else None
        self.lidar_callback_url = client_config['lidar_callback_url'] if 'lidar_callback_url' in client_config else None
        self.ground_camera_callback_url = client_config['ground_camera_callback_url'] if 'ground_camera_callback_url' in client_config else None
        self.imu_gps_callback_url = client_config['imu_gps_callback_url'] if 'imu_gps_callback_url' in client_config else None
        self.run_complete_callback_url = client_config['run_complete_callback_url'] if 'run_complete_callback_url' in client_config else None
        self.run_start_callback_url = client_config['run_start_callback_url'] if 'run_start_callback_url' in client_config else None

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
            self.client_id = client_config['client_id']

        if 'debug_draw' in client_config and client_config['debug_draw']:
            self.orchestrator.set_debug_draw_enabled(True)
        else:
            self.orchestrator.set_debug_draw_enabled(False)

        self.start_time = datetime.datetime.now(tz=self.timezone)
        self.end_time = None

        self.run_id = self.database_client.create_simulation_run(self.start_time,
                                                                  self.server_config,
                                                                  client_config,
                                                                  client_config, # for now, concrete and logical config are the same thing
                                                                  self.client_id,
                                                                  self.simulation_instance_manager.simulation_execution_id,
                                                                  self.orchestrator.cones,
                                                                  self.orchestrator.start_pose,
                                                                  self.orchestrator.goal_point)


        self.monitor_thread.start()

        # TODO: hack to ensure that orchestrator starts.
        time.sleep(0.5)

        for callback in self.background_threads:
            callback.start()

        if (self.run_start_callback_url is not None):
            begin_status = self.orchestrator.get_run_summary()
            post_data = json.dumps(begin_status, default=str)
            unposted = json.loads(post_data)
            requests.post(self.run_start_callback_url, json=unposted)

    def set_control_signals(self, left_throttle, right_throttle):
        if (self.is_running):
            with SimulationRun.LockedSection(self) as l:
                self.client.drive(left_throttle, right_throttle)
                return True

    def cancel(self):
        self.exception_queue.put(('The operation was cancelled by the user.', None))

    def get_status(self):
        return self.last_status

    def __main_monitor_thread(self):
        try:
            now = datetime.datetime.now(tz=self.timezone)

            with SimulationRun.LockedSection(self) as l:
                self.orchestrator.start_new_run(self.client)

            time.sleep(2)
            self.last_status = self.orchestrator.get_run_summary()
            is_run_complete = self.last_status['runComplete']

            while(is_run_complete):
                time.sleep(0.1)
                with SimulationRun.LockedSection(self) as l:
                    self.orchestrator.run_tick(self.client)
                is_run_complete = self.orchestrator.get_run_summary()['runComplete']


            visited_cone_ids = set()
            while(not is_run_complete):
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

                with SimulationRun.LockedSection(self) as l:
                    self.orchestrator.run_tick(self.client)

                self.last_status = self.orchestrator.get_run_summary()
                is_run_complete = self.last_status['runComplete']
        except Exception as e:
            self.error = str(e)
            self.error_stack_trace = traceback.format_exc()

        self.is_running = False
        
        if (self.last_status is None):
            self.last_status = {'status': 'completed'}
        else:
            self.last_status['isCompleted'] = True

        # Attempt to stop the car.
        # It's no big deal if it doesn't stop.
        visited = self.orchestrator.goal_point.visited
        closest_distance = self.orchestrator.goal_point.closest_distance

        try:
            with SimulationRun.LockedSection(self) as l:
                self.client.drive(0, 0)
                self.orchestrator.clean_up_run(self.client)
        except Exception as e:
            pass

        self.end_time = datetime.datetime.now(tz=self.timezone)
        self.database_client.complete_simulated_run(self.run_id,
                                                    self.end_time,
                                                    self.error,
                                                    self.error_stack_trace,
                                                    closest_distance,
                                                    visited)

        for background_thread in self.background_threads:
            background_thread.join()

        if (self.run_complete_callback_url is not None):
            post_data = {}
            post_data['end_time'] = self.end_time
            post_data['error'] = self.error
            post_data['error_stack_trace'] = self.error_stack_trace
            post_data['goal_visited'] = self.orchestrator.goal_point.visited

            post_data_str = json.dumps(post_data, default=str)
            post_data_out = json.loads(post_data_str)

            requests.post(self.run_complete_callback_url, json=post_data_out)
        
        self.is_complete = True

    def __bot_location_monitor_bg_worker(self):
        try:
            now = datetime.datetime.now(tz=self.timezone)
            while(self.is_running):
                next_now = now + datetime.timedelta(seconds = 1.0 / 2.0) # 2 hz
                
                with SimulationRun.LockedSection(self) as l:
                    response = self.client.simGetVehiclePose()

                self.database_client.insert_bot_pose(self.run_id,
                                                     now,
                                                     response.position,
                                                     response.orientation)

                now = next_now
                self.__pause_until(now)
        except Exception as e:
            self.exception_queue.put((str(e), traceback.format_exc()))
            raise

    def __publish_depth_image_bg_worker(self):
        try:
            now = datetime.datetime.now(tz=self.timezone)
            while (self.is_running):
                now += datetime.timedelta(seconds = 1.0 / 30.0) # 30 FPS

                with SimulationRun.LockedSection(self) as l:
                    response = self.client.simGetImages([
                        at.ImageRequest('Xtion', at.ImageType.Scene, pixels_as_float=False, compress=False),
                        at.ImageRequest('Xtion', at.ImageType.DepthPlanner, pixels_as_float=True, compress=False)]) # Misspelled "Planar"

                # simulator returns depths in meters. 
                # xtion returns in mm
                # range of use is between 0.8m and 3.5m. (https://www.asus.com/3D-Sensor/Xtion_PRO/specifications/)
                # code golfed for performance...
                post_data = response[0].height.to_bytes(4, 'big') \
                        + response[0].width.to_bytes(4, 'big') \
                        + response[0].image_data_uint8 \
                        + np.clip(np.array(response[1].image_data_float, dtype=np.float32) * 1000.0, 800, 3500).astype(np.uint16).tobytes(order=None)

                response = requests.post(self.depth_image_callback_url, data=post_data)

                self.__pause_until(now)
        except Exception as e:
            self.exception_queue.put((str(e), traceback.format_exc()))
            raise

    def __publish_lidar_bg_worker(self):
        try:
            now = datetime.datetime.now(tz=self.timezone)
            data_buf = np.zeros((360), dtype=np.float32)
            first_buf = True
            while (self.is_running):
                now += datetime.timedelta(seconds= 1.0 / 5.0) # 5 Hz

                with SimulationRun.LockedSection(self) as l:
                    response = self.client.getLidarData('Lidar')

             

                should_send = False
                for i in range(0, len(response.point_cloud), 3):
                    x = response.point_cloud[i]
                    y = response.point_cloud[i+1]
                    ang = np.arctan2(y, x) * 180.0 / np.pi
                    if (ang < 0):
                        ang += 360.0

                    bucket = int(ang)
                    data_buf[bucket] = np.linalg.norm(response.point_cloud[i:i+3]) * 10.0 # Response from AirSim comes in cm, lidar is in mm

                if (first_buf):
                    first_buf = False
                else:
                    post_data = np.clip(data_buf, 0, 6000).tobytes(order=None)
                    response = requests.post(self.lidar_callback_url, data=post_data)

                self.__pause_until(now)

        except Exception as e:
            self.exception_queue.put((str(e), traceback.format_exc()))
            raise


    def __publish_ground_image_bg_worker(self):
        try:
            now = datetime.datetime.now(tz=self.timezone)
            while (self.is_running):
                now += datetime.timedelta(seconds = 1.0 / 30.0) # 30 FPS

                with SimulationRun.LockedSection(self) as l:
                    response = self.client.simGetImages([
                        at.ImageRequest('Ground', at.ImageType.Scene, pixels_as_float=False, compress=False)])

                post_data = response[0].height.to_bytes(4, 'big') \
                          + response[0].width.to_bytes(4, 'big') \
                            + response[0].image_data_uint8

                response = requests.post(self.ground_camera_callback_url, data=post_data)

                self.__pause_until(now)
        except Exception as e:
            self.exception_queue.put((str(e), traceback.format_exc()))
            raise

    def __publish_imu_gps_bg_worker(self):
        try:
            now = datetime.datetime.now(tz=self.timezone)

            while(self.is_running):
                now += datetime.timedelta(seconds = 1.0 / 20.0) # 20 hz

                with SimulationRun.LockedSection(self) as l:
                    response = self.client.readSensors()

                post_data = {}
                post_data['imu_x'] = response['readings']['BodyImu']['IMU-Lin-x']
                post_data['imu_y'] = response['readings']['BodyImu']['IMU-Lin-y']
                post_data['imu_z'] = response['readings']['BodyImu']['IMU-Lin-z']
                post_data['imu_roll'] = response['readings']['BodyImu']['IMU-Ang-roll']
                post_data['imu_pitch'] = response['readings']['BodyImu']['IMU-Ang-pitch']
                post_data['imu_yaw'] = response['readings']['BodyImu']['IMU-Ang-yaw']

                post_data['latitude'] = response['readings']['GPS']['GPS-Loc-latitude']
                post_data['longitude'] = response['readings']['GPS']['GPS-Loc-longitude']
                post_data['altitude'] = response['readings']['GPS']['GPS-Loc-altitude']

                post_data['mag_x'] = response['readings']['Magnetometer']['Mag-Vec-x']
                post_data['mag_y'] = response['readings']['Magnetometer']['Mag-Vec-y']
                post_data['mag_z'] = response['readings']['Magnetometer']['Mag-Vec-z']

                response = requests.post(self.imu_gps_callback_url, json=post_data)

                self.__pause_until(now)
        except Exception as e:
            self.exception_queue.put((str(e), traceback.format_exc()))
            raise

    # Copied from pause package. 
    # Cannot use package directly because their timezone is not tz aware.
    def __pause_until(self, end):
        while True:
            now = datetime.datetime.now(tz=self.timezone)
            diff = (end - now).total_seconds()

            if diff <= 0.001: # Allow a bit of buffer to avoid spamming calls close to end
                break
            else:
                time.sleep(diff / 2)
