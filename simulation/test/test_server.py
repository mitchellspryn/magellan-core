import json
import requests
import time

def main():
    server_port = 5000
    print('Reading config data...')
    with open('TestOrchestrationConfiguration.json', 'r') as f:
        config = json.loads(f.read())

    print('Checking to see if server is alive...')
    server_url = 'http://127.0.0.1:{0}/'.format(server_port)

    response = requests.get(server_url + 'ping')
    if (response.status_code != 200):
        print('FAILED: status code = {0}.'.format(status_code))
    if (response.content.decode('utf-8') != 'pong'):
        print('FAILED: expected response "pong", got "{0}".'.format(response.content))
        
    print ('Ping successful. Creating new experiment...')
    config['depth_image_callback_url'] = None
    config['lidar_callback_url'] = None
    config['ground_camera_callback_url'] = None
    config['imu_gps_callback_url'] = None
    config['run_complete_callback_url'] = None
    config['client_id'] = 'test'
    config['debug_draw'] = False
    
    response = requests.post(server_url + 'runs', json=config)
    if (response.status_code != 200):
        print('FAILED: status_code = {0}'.format(response.status_code))
    
    print('Created experiment successfully.')
    
    for i in range(0, 2, 1):
        if (i == 1):
            print('Driving forward')
            data = {}
            data['left_throttle'] = 0.6
            data['right_throttle'] = 0.6
            response = requests.post(server_url + 'control', json=data)
            if (response.status_code != 200):
                print('FAILED. status_code = {0}'.format(response.status_code))
            
        
        print('Waiting 3 seconds...')
        time.sleep(3)
        
        print('Getting current status...')
        response = requests.get(server_url + 'runs')
        if (response.status_code != 200):
            print('FAILED: status code = {0}'.format(response.status_code))
        
        content = json.loads(response.content.decode('utf-8'))
        print('Status: ')
        print(content)
        
    print('Cancelling experiment.')
    response = requests.delete(server_url + 'runs')
    if (response.status_code != 200):
        print('Failed. Status code = {0}'.format(response.status_code))
    
    print('Test complete.')
        
if __name__ == '__main__':
    main()



