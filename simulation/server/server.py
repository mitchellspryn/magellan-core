import json

import simulation_run
import simulation_instance_manager

import flask
app = flask.Flask(__name__)

@app.route('/status')
def status_page():
    return 'Hello, world!'

@app.route('/runs', methods=['POST'])
def new_run():
    pass

@app.route('/runs', methods=['GET'])
def status():
    pass

@app.route('/control', methods=['POST'])
def set_control_signals():
    pass

@app.route('/runs', methods=['DELETE'])
def cancel_run():
    pass

sim_instance_manager = None

if __name__ == '__main__':
    try:
        sim_instance_manager = simulation_instance_manager.SimulationInstanceManager()
        app.run()
    finally:
        if (sim_instance_manager is not None):
            sim_instance_manager.finalize()
            sim_instance_manager = None

