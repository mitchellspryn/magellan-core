import json
import threading
import time
import requests

import simulation_run
import server_config
import simulation_database.simulation_database_manager as simulation_database_manager
import simulator_instance_manager

import flask
app = flask.Flask(__name__)

@app.route('/ping', methods=['GET'])
def status_page():
    return 'pong', 200

@app.route('/control', methods=['POST'])
def set_control_signals():
    global current_simulation_run
    if (current_simulation_run is None):
        return json.dumps({'error': 'No active simulation run'}), 400

    data = flask.request.json

    if (data is None):
        return json.dumps({'error': 'Json not supplied.'}), 400

    if ('left_throttle' not in data or 'right_throttle' not in data):
        return json.dumps({'error': 'Json missing left_throttle or right_throttle.'}), 400

    if (data['left_throttle'] > 1 or data['left_throttle'] < -1):
        return json.dumps({'error': 'left_throttle is {0}, which is outside the range [-1, 1].'.format(data['left_throttle'])}), 400

    if (data['right_throttle'] > 1 or data['right_throttle'] < -1):
        return json.dumps({'error': 'right_throttle is {0}, which is outside the range [-1, 1].'.format(data['right_throttle'])}), 400

    if (not current_simulation_run.set_control_signals(data['left_throttle'], data['right_throttle'])):
        return json.dumps({'error': 'Could not set control signals.'}), 400

    return json.dumps({}), 200

@app.route('/runs', methods=['POST'])
def new_run():
    global current_simulation_run
    global sim_instance_manager
    global database_manager
    global global_server_config

    if (current_simulation_run is not None and current_simulation_run.is_complete):
        del current_simulation_run
        current_simulation_run = None

    if (current_simulation_run is not None):
        return json.dumps({'error': 'Simulation currently in progress.'}), 400

    create_request = flask.request.json

    try:
        current_simulation_run = simulation_run.SimulationRun(create_request,
                                                              sim_instance_manager,
                                                              database_manager,
                                                              global_server_config)
        
        return json.dumps({}), 200
    except Exception as e:
        raise
        # return json.dumps({'error': 'Cannot start simulation: {0}'.format(str(e))}), 400

@app.route('/runs', methods=['GET'])
def status():
    global current_simulation_run
    if (current_simulation_run is None):
        return json.dumps({'status': 'No simulation in progress.'}), 200

    # Default is needed for datetime types.
    return json.dumps(current_simulation_run.last_status, default=str), 200

@app.route('/runs', methods=['DELETE'])
def cancel_run():
    global current_simulation_run
    if (current_simulation_run is None):
        return json.dumps({'error': 'No simulation in progress.'}), 400

    current_simulation_run.cancel()
    
    wait_count = 0
    while(current_simulation_run.is_running):
        wait_count += 1
        if (wait_count > 100):
            return json.dumps({'error': 'Could not stop simulation. Run.cancel() timed out.'}), 500
        time.sleep(0.05)

    del current_simulation_run
    current_simulation_run = None
    return json.dumps({}), 200

sim_instance_manager = None
current_simulation_run = None
global_server_config = None
database_manager = None

if __name__ == '__main__':
    try:
        global_server_config = server_config.ServerConfig('server/server_config.json')
        database_manager = simulation_database_manager.SimulationDatabaseManager(global_server_config.db_user,
                                                                         global_server_config.db_password,
                                                                         global_server_config.db_host,
                                                                         global_server_config.db_port,
                                                                         global_server_config.db_database)

        sim_instance_manager = simulator_instance_manager.SimulatorInstanceManager(global_server_config.simulator_path,
                                                                                   global_server_config.simulator_log_path,
                                                                                   global_server_config.simulator_delete_log_on_success)

        app.run(host='0.0.0.0')
    finally:
        if (sim_instance_manager is not None):
            sim_instance_manager.finalize()
            del sim_instance_manager
            sim_instance_manager = None

