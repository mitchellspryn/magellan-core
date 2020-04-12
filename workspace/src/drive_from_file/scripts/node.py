#!/usr/bin/env python3
import sys
import argparse
import rospy
import time
import os

from magellan_messages.msg import MsgMagellanDrive

def parse_command_line_args():
    filtered_args = rospy.myargv(sys.argv)

    parser = argparse.ArgumentParser(description='Command line arguments for drive node')
    parser.add_argument('--drive-file', dest='drive_file', type=str, required=True, help='The file to read for driving')

    return parser.parse_args(filtered_args[1:])

def read_drive_file(filename):
    drive_commands = []
    line_number = 1

    file_path = os.path.join(os.getcwd(), 'drive_files', filename)
    if not (os.path.isfile(file_path)):
        rospy.logfatal('cannot find requested drive file {0}.'.format(file_path))
        return None

    with open(file_path, 'r') as f:
        for line in f:
            split_line = line.split(' ')
            if len(split_line) != 3:
                rospy.logfatal('Improper line found in drive file {0} at line {1}: "{2}". Expected three figures separated by space.'.format(filename, line_number, line))
                return None
            
            try:
                left_wheel_speed = float(split_line[0])
            except:
                rospy.logfatal('Improper line found in drive file {0} at line {1}: "{2}". Expected first field to be float.'.format(filename, line_number, line))
                return None

            try:
                right_wheel_speed = float(split_line[1])
            except:
                rospy.logfatal('Improper line found in drive file {0} at line {1}: "{2}". Expected second field to be float.'.format(filename, line_number, line))
                return None

            try:
                sleep_time = float(split_line[2])
            except:
                rospy.logfatal('Improper line found in drive file {0} at line {1}: "{2}". Expected third field to be float.'.format(filename, line_number, line))
                return None

            if (abs(left_wheel_speed) > 100):
                rospy.logfatal('Improper left wheel speed (field 1) found in drive file {0} at line {1}: "{2}". Expected to be on the range [-100, 100], found {3}'.format(filename, line_number, line, left_wheel_speed))
                return None

            if (abs(right_wheel_speed) > 100):
                rospy.logfatal('Improper right wheel speed (field 2) found in drive file {0} at line {1}: "{2}". Expected to be on the range [-100, 100], found {3}'.format(filename, line_number, line, right_wheel_speed))
                return None

            if (sleep_time <= 0):
                rospy.logfatal('Improper sleep_time (field 3) found in drive file {0} at line {1}: "{2}". Expected to be >0, found value {3}'.format(filename, line_number, line, sleep_time))
                return None

            drive_commands.append( (left_wheel_speed, right_wheel_speed, sleep_time) )
            line_number += 1

    return drive_commands

def run_drive_commands(drive_commands, publisher):
    for command_index in range(0, len(drive_commands), 1):
        drive_command = drive_commands[command_index]

        rospy.logerr('Sending command ({0}, {1}) for {2} seconds. Command {3}/{4}'.format(drive_command[0], drive_command[1], drive_command[2], command_index, len(drive_commands)))

        message = MsgMagellanDrive()
        message.left_throttle = drive_command[0]
        message.right_throttle = drive_command[1]
        message.header.stamp = rospy.Time.now()
        
        publisher.publish(message)

        time.sleep(drive_command[2])

    rospy.loginfo('Stopping bot.')
    final_message = MsgMagellanDrive()
    final_message.left_throttle = 0
    final_message.right_throttle = 0
    final_message.header.stamp = rospy.Time.now()

    publisher.publish(final_message)

def main():
    rospy.init_node('drive_from_file')

    args = parse_command_line_args()
    publisher = rospy.Publisher('output_topic', MsgMagellanDrive, queue_size = 100)
    
    drive_commands = read_drive_file(args.drive_file)

    if (drive_commands is None or len(drive_commands) == 0):
        rospy.logfatal('There was a problem reading the drive file.')
    else:
        rospy.logerr('Drive file loaded. {0} commands read. Giving 10 seconds for hardware to initialize.'.format(len(drive_commands)))
        time.sleep(10)
        rospy.logerr('Running commands.')
        run_drive_commands(drive_commands, publisher)

    rospy.signal_shutdown('Drive file replay complete.')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
