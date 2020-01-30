import json
import requests

def main():
    server_port = 5000
    print('Reading config data...')
    with open('TestOrchestrationConfiguration.json', 'r') as f:
        config = json.loads(f.read())

    print('Checking to see if server is alive')
    server_url = 'http://127.0.0.1:{0}/'.format(server_port)

    response = requests.get(server_url + 'ping')
    if (response.status_code != 200):
        print('FAILED: status code = {0}.'.format(status_code))
    if (response.content.decode('utf-8') != 'pong'):
        print('FAILED: expected response "pong", got "{0}".'.format(response.content))

if __name__ == '__main__':
    main()



