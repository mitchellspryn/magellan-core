import os
import subprocess
import uuid

class SimulatorInstanceManager(object):
    """
    Manages the running instance of the simulator
    """
    def __init__(self, simulator_path, simulator_log_path, delete_log_on_success):
        self.simulator_path = simulator_path
        self.simulator_log_path = simulator_log_path
        self.delete_log_on_success = delete_log_on_success
        self.simulation_execution_id = str(uuid.uuid4())
        
        self.__simulator_popen_obj = self.__start_simulator()

    def get_simulator_pid(self):
        if self._simulator_popen_obj is not None:
            return self.__simulator_popen_obj.pid

    def get_log_path(self):
        return os.path.join(self.simulator_log_path, '{0}.log'.format(self.simulator_execution_id)).replace('\\', '/')

    def is_simulation_alive(self):
        return (self.__simulator_popen_obj.poll() == None)

    def finalize(self):
        if (self.simulator_pid is not None):
            self.__kill_simulator_process()
            self.simulator_pid = None

    def __start_simulator(self):
        args = [self.simulator_path, 'ABSLOG={0}'.format(self.get_log_path())]
        return subprocess.Popen(args)

    def __kill_simulator_process(self):
        if (self.__simulator_popen_obj is None):
            return

        is_termination_graceful = self.is_simulation_alive()
        self.__simulator_popen_obj.kill()

        if (is_termination_graceful and self.delete_log_on_success and os.path.exists(self.get_log_path()):
            os.remove(self.get_log_path())

        self.__simulator_popen_obj = None






