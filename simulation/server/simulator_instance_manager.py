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
        
        self.simulator_exe_name = self.simulator_path.split('/')[-1]

        self.__kill_simulator_process(force=True)
        
        self.__simulator_popen_obj = self.__start_simulator()
    
    def get_simulator_pid(self):
        """
        NOTE: For some reason, 2 simulator processes seem to spawn...
              Not sure which PID gets returned.
              Or why there are two.
        """
        if self.__simulator_popen_obj is not None:
            return self.__simulator_popen_obj.pid
        return None

    def get_log_path(self):
        return os.path.join(self.simulator_log_path, '{0}.log'.format(self.simulation_execution_id)).replace('\\', '/')

    def is_simulation_alive(self):
        if (self.__simulator_popen_obj is None):
            return False
        return (self.__simulator_popen_obj.poll() == None)

    def finalize(self):
        if (self.__simulator_popen_obj is not None):
            self.__kill_simulator_process()
            self.__simulator_popen_obj = None

    def __start_simulator(self):
        args = [self.simulator_path, 'ABSLOG={0}'.format(self.get_log_path())]
        return subprocess.Popen(args)

    def __kill_simulator_process(self, force = False):
        if (not force and self.__simulator_popen_obj is None):
            return

        is_termination_graceful = False
        if (not force):
            is_termination_graceful = self.is_simulation_alive()
        
        # This doesn't seem to work.
        # self.__simulator_popen_obj.kill()

        # Time to use a bigger hammer
        args = ['taskkill', '/FI', 'IMAGENAME eq {0}'.format(self.simulator_exe_name), '/f']
        subprocess.call(args)

        if (is_termination_graceful and self.delete_log_on_success and os.path.exists(self.get_log_path())):
            os.remove(self.get_log_path())

        self.__simulator_popen_obj = None
