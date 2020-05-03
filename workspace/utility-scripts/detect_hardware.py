#!/usr/bin/env python3
import serial
import subprocess
import argparse
import colorama
import os
import shutil
import uuid

find_zed = True
try:
    import pyzed.sl as sl
except:
    find_zed = False

NODES_OF_INTEREST = ['ttyACM0', 'ttyACM1', 'ttyACM2', 'ttyACM3', 'video1', 'video2', 'ttyUSB0']

def parse_args():
    parser = argparse.ArgumentParser(description='Detects connected hardware, optionally generating the roslaunch files.')

    parser.add_argument('--generate-launch-files', action='store_true', dest='generate_launch_files', required=False)
    parser.add_argument('--strict', action='store_true', dest='strict', required=False)

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

def get_rplidar_node(listings):
    for potential_node in [n for n in NODES_OF_INTEREST if 'USB' in n]:
        if potential_node in listings['udevadm_listings']:
            listing = listings['udevadm_listings'][potential_node]

            # On the jetson, udevadm emits an extra line with "gps0" in it. 
            if    (len(listing) >= 3 and 'Silicon_Labs_CP2102' in listing[2]) \
               or (len(listing) >= 4 and 'Silicon_Labs_CP2102' in listing[3]):
                return potential_node

    return None

def get_arduino_node(listings):
    for potential_node in [n for n in NODES_OF_INTEREST if 'ACM' in n]:
        if potential_node in listings['udevadm_listings']:
            listing = listings['udevadm_listings'][potential_node]
            if len(listing) >= 3 and 'Arduino' in listing[2]:
                return potential_node

    return None

# Not actually a node, but check if it's available.
def get_zed_node(listings):
    global find_zed
    result = None

    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.sdk_verbose = False

    err = zed.open(init_params)
    if err == sl.ERROR_CODE.SUCCESS:
        result = zed.get_camera_information().serial_number
        zed.close()

    if ((result == None) and (not find_zed)):
        result = 'disabled'

    return result

def get_video_node(listings):
    # TODO: is there a better way to distinguish the logitech camera from something else?
    # For now, we don't need to.

    if 'video1' in listings['udevadm_listings']:
        return 'video1'
    elif 'video2' in listings['udevadm_listings']:
        return 'video2'

    return None

def get_mass_storage_path():
    # Attempt to write a text file.
    # If it succeeds, then the drive is attached.
    try:
        text_to_write = str(uuid.uuid4())
        with open('/media/usb-drive/build.txt', 'w') as f:
            f.write(text_to_write)

        with open('/media/usb-drive/build.txt', 'r') as f:
            text = f.read()
            if (text != text_to_write):
                return None

        os.remove('/media/usb-drive/build.txt')
        return '/media/usb-drive/'
    except:
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
            print('???', end='')
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
                replaced = in_file.read()
                

                if 'arduino' in node_mappings:
                    replaced = replaced.replace('{arduino_device_node}', '/dev/{0}'.format(node_mappings['arduino']))

                if 'lidar' in node_mappings:
                    replaced = replaced.replace('{lidar_device_node}', '/dev/{0}'.format(node_mappings['lidar']))

                if 'left_motor' in node_mappings:
                    replaced = replaced.replace('{left_motor_device_node}', '/dev/{0}'.format(node_mappings['left_motor']))

                if 'right_motor' in node_mappings:
                    replaced = replaced.replace('{right_motor_device_node}', '/dev/{0}'.format(node_mappings['right_motor']))

                if 'rplidar' in node_mappings:
                    replaced = replaced.replace('{rplidar_device_node}', '/dev/{0}'.format(node_mappings['rplidar']))

                if 'ground_camera' in node_mappings:
                    replaced = replaced.replace('{ground_camera_index}', node_mappings['ground_camera'].replace('video', ''))

                if 'mass_storage' in node_mappings:

                    # If mass storage is present, use it for writing rosbags.
                    # Otherwise, write to local storage.
                    if node_mappings['mass_storage'] is not None:
                        replacement_text = node_mappings['mass_storage']
                    else:
                        replacement_text = ''

                    replaced = replaced.replace('{rosbag_output_dir}', replacement_text)

                out_file.write(replaced)

def main():
    args = parse_args()

    print('Searching for connected devices...')
    
    listings = get_listings()

    node_mappings = {}
    node_mappings['arduino'] = get_arduino_node(listings)
    #node_mappings['lidar'] = get_lidar_node(listings)
    node_mappings['rplidar'] = get_rplidar_node(listings)
    node_mappings['left_motor'] = get_motor_controller_node(listings, 130)
    node_mappings['right_motor'] = get_motor_controller_node(listings, 128)
    node_mappings['ground_camera'] = get_video_node(listings)
    node_mappings['mass_storage'] = get_mass_storage_path()
    node_mappings['zed'] = get_zed_node(listings)

    # TODO: list the xtion

    print_results(node_mappings)

    if (args.generate_launch_files):
        if not all(node_mappings.values()):
            if args.strict:
                print(colorama.Fore.RED, end='')
                print('Cannot generate launch files, as some connected devices could not be found. Please check for hardware connectivity.')
                print(colorama.Style.RESET_ALL)
                return
            else:
                print(colorama.Fore.RED, end='')
                print('*************')
                print('* WARNING   *')
                print('*************')
                print()
                print('Not all devices could be found. Not all launch files will be valid.')
                print(colorama.Style.RESET_ALL, end='')

        refresh_launch_files(node_mappings)
        print('Launch files have been regenerated.')
    
if __name__ == '__main__':
    main()
