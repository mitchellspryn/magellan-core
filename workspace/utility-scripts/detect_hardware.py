import serial
import subprocess
import argparse
import colorama
import os
import shutil

NODES_OF_INTEREST = ['ttyACM0', 'ttyACM1', 'ttyACM2', 'ttyACM3', 'video1']

def parse_args():
    parser = argparse.ArgumentParser(description='Detects connected hardware, optionally generating the roslaunch files.')

    parser.add_argument('--generate-launch-files', action='store_true', dest='generate_launch_files', required=False)

    args = parser.parse_args()

    return args

def get_udevadm_listing(node):
    udevadm_listing = subprocess.run(['udevadm', 'info', '--name=/dev/{0}'.format(node)], stdout=subprocess.PIPE)

    out_str = udevadm_listing.stdout.decode('utf-8')

    return [o for o in out_str.split('\n') if len(o) > 0]

def get_listings():
    dev_listing = subprocess.run(['ls', '/dev'], stdout=subprocess.PIPE)
    lsusb_listing = subprocess.run(['lsusb'], stdout=subprocess.PIPE)

    visible_dev_nodes = set([v for v in dev_listing.stdout.decode('utf-8').split('\n') if len(v) > 0])
    visible_lsusb_devices = set([v for v in lsusb_listing.stdout.decode('utf-8').split('\n') if len(v) > 0])

    output = {}
    output['dev_nodes'] = visible_dev_nodes
    output['lsusb_devices'] = visible_lsusb_devices

    udevadm_listings = {}
    for node_of_interest in NODES_OF_INTEREST:
        if node_of_interest in visible_dev_nodes:
            udevadm_listings[node_of_interest] = get_udevadm_listing(node_of_interest)

    output['udevadm_listings'] = udevadm_listings

    return output

def get_motor_controller_node(listings, address):
    for potential_node in [n for n in NODES_OF_INTEREST if 'ACM' in n]:
        if potential_node in listings['udevadm_listings']:
            listing = listings['udevadm_listings'][potential_node]
            if len(listing) < 3 or not ('Roboclaw' in listing[2]):
                continue
            
            # Try to communicate with the device.
            # Send a command to read the firmware version.
            # If the address is wrong, we won't get a response.
            result = []
            with serial.Serial('/dev/{0}'.format(potential_node), timeout=3) as s:
                s.write([address, 21])
                result = s.read(20)

            if (len(result) > 0 and result.decode('ascii').startswith('USB Roboclaw')):
                return potential_node

    return None

def get_lidar_node(listings):
    for potential_node in [n for n in NODES_OF_INTEREST if 'ACM' in n]:
        if potential_node in listings['udevadm_listings']:
            listing = listings['udevadm_listings'][potential_node]
            if len(listing) >= 3 and 'Teensyduino' in listing[2]:
                return potential_node

    return None

def get_arduino_node(listings):
    for potential_node in [n for n in NODES_OF_INTEREST if 'ACM' in n]:
        if potential_node in listings['udevadm_listings']:
            listing = listings['udevadm_listings'][potential_node]
            if len(listing) >= 3 and 'Arduino' in listing[2]:
                return potential_node

    return None

def get_video_node(listings):
    # TODO: is there a better way to distinguish the logitech camera from something else?
    # For now, we don't need to.

    if 'video1' in listings['udevadm_listings']:
        return 'video1'

    return None

def print_results(node_mappings):
    print(colorama.Fore.GREEN)
    print('*****************************')
    print('*  MAPPING RESULTS          *')
    print('*****************************')
    print(colorama.Style.RESET_ALL)

    for device_name in sorted([k for k in node_mappings]):
        print('{0}: '.format(device_name).ljust(20), end='')
        if (node_mappings[device_name] is not None):
            print(colorama.Fore.GREEN, end='')
            print(node_mappings[device_name], end='')
        else:
            print(colorama.Fore.RED, end='')
            print('???')
        print(colorama.Style.RESET_ALL, end='')
        print()

    print()

def refresh_launch_files(node_mappings):
    directories = [f for f in os.listdir('.') if os.path.isdir(f)]
    launch_template_dir = 'launch_template'
    launch_dir = 'launch'
    if launch_template_dir not in directories:
        print(colorama.Fore.RED)
        print('Cannot generate launch files because directory launch_template is not found.')
        print(colorama.Style.RESET_ALL)
        return

    if launch_dir in directories:
        shutil.rmtree(launch_dir)

    os.makedirs(launch_dir)

    launch_templates = [f for f in os.listdir(launch_template_dir) if os.path.isfile(os.path.join(launch_template_dir, f)) and f.endswith('.launchtemplate')]
    for template in launch_templates:
        with open(os.path.join(launch_template_dir, template), 'r') as in_file:
            with open(os.path.join(launch_dir, template[:-15] + '.launch'), 'w') as out_file:
                text = in_file.read()

                replaced = text.replace('{arduino_device_node}', '/dev/{0}'.format(node_mappings['arduino']))
                replaced = replaced.replace('{lidar_device_node}', '/dev/{0}'.format(node_mappings['lidar']))
                replaced = replaced.replace('{left_motor_device_node}', '/dev/{0}'.format(node_mappings['left_motor']))
                replaced = replaced.replace('{right_motor_device_node}', '/dev/{0}'.format(node_mappings['right_motor']))
                replaced = replaced.replace('{ground_camera_index}', '1')

                out_file.write(replaced)

def main():
    args = parse_args()

    print('Searching for connected devices...')
    
    listings = get_listings()

    node_mappings = {}
    node_mappings['arduino'] = get_arduino_node(listings)
    node_mappings['lidar'] = get_lidar_node(listings)
    node_mappings['left_motor'] = get_motor_controller_node(listings, 130)
    node_mappings['right_motor'] = get_motor_controller_node(listings, 128)
    node_mappings['ground_camera'] = get_video_node(listings)

    # TODO: list the xtion

    print_results(node_mappings)

    if (args.generate_launch_files):
        if not all(node_mappings.values()):
            print(colorama.fore.RED, end='')
            print('Cannot generate launch files, as some connected devices could not be found. Please check for hardware connectivity.')
            print(colorama.style.RESET_ALL)
        else:
            refresh_launch_files(node_mappings)
            print('Launch files have been regenerated.')
    
if __name__ == '__main__':
    main()
