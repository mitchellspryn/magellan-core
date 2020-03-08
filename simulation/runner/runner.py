import os
import sys
import json
import requests
import time

TESTS_TO_RUN = ['tests/basic.json']

def find_attached_hil_rig():
    potential_hil_rigs = []
    potential_hil_rigs.append( ('http://192.168.0.20:55555', 'laptop') )
    potential_hil_rigs.append( ('http://192.168.0.10:55555', 'robot') )

    for potential_hil_rig in potential_hil_rigs:
        uri = potential_hil_rig[0]
        hil_rig_name = potential_hil_rig[1]

        try:
            response = requests.get(uri + '/ping')
            if (response.status_code == 200 and response.text.strip().lower() == 'pong'):
                return (uri, hil_rig_name)
        except:
            pass

    return (None, None)

def find_simulation_server():
    try:
        test_uri = 'http://192.168.0.30:5000'
        response = requests.get(test_uri + '/ping')
        if response.status_code == 200 and response.text.strip().lower() == 'pong':
            return test_uri
    except:
        return None

def run_test(filepath, hil_server_uri, simulation_server_uri):
    print('Initializing test...')
    with open(filepath, 'r') as f:
        test_config = json.loads(f.read())

    test_config['depth_image_callback_url'] = hil_server_uri + '/xtion'
    test_config['lidar_callback_url'] = hil_server_uri + '/lidar'
    test_config['ground_camera_callback_url'] = hil_server_uri + '/ground_camera'
    test_config['imu_gps_callback_url'] = hil_server_uri + '/imu_gps'
    test_config['run_start_callback_url'] = hil_server_uri + '/run_start'
    test_config['run_complete_callback_url'] = hil_server_uri + '/run_complete'

    response = requests.post(simulation_server_uri + '/runs', json=test_config)
    if (response.status_code != 200):
        print('Could not start run. Response from POST to /runs is {0}: {1}'.format(response.status_code, response.text))
        return

    print('Test started. Waiting for a few seconds to begin status check loop.')
    time.sleep(5)
    print('Checking status until termination.')
    while True:
        response = requests.get(simulation_server_uri + '/runs')
        if (response.status_code == 200):
            data = json.loads(response.text)
            if ('runComplete' in data and data['runComplete']):
                break
        else:
            print('Warning: GET request to /runs returns {0}.'.format(response.status_code))
        
        time.sleep(5)

    print('Test complete. Check database for results.')

def main():
    print('Verifying that server is enabled...', end='')
    simulation_server_uri = find_simulation_server()
    if simulation_server_uri is None:
        print('')
        print('ERROR: could not connect to server.')
        print('Ensure that it is up and running.')
        return

    print('Success.')
    print('Finding attached HIL rig...', end='')
    hil_uri, hil_name = find_attached_hil_rig()
    if (hil_uri is None or hil_name is None):
        print('')
        print('Could not connect to hil rig. Please verify that one is connected and visible on the network.')
        return

    print('Found HIL rig named {0} at {1}.'.format(hil_name, hil_uri))
    print('Beginning tests...')
    for test in TESTS_TO_RUN:
        print('***Running test {0}***'.format(test))
        run_test(test, hil_uri, simulation_server_uri)

    print('Tests complete.')

if __name__ == '__main__':
    main()
