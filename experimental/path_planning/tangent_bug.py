import numpy as np
import pandas as pd
import cv2
import argparse

class Map(object):
    def __init__(self, map_grid, start, goal):
        self.start = start
        self.goal = goal
        self.map_grid = map_grid
        self.height = map_grid.shape[0]
        self.width = map_grid.shape[1]

    def is_obstacle(self, y, x):
        return self.map_grid[y][x] == 1

    def is_start(self, y, x):
        return ((y == self.start[0]) and (x == self.start[1]))

    def is_goal(self, y, x):
        return ((y == self.goal[0]) and (x == self.goal[1]))

    def is_clear(self, y, x):
        return self.map_grid[y][x] == 0

def read_map(file_name):
    data = []
    start_position = None
    goal_position = None

    row_num = -1
    with open(file_name, 'r') as f:
        for line in f:
            row_num += 1
            col_num = -1
            row = []
            for char in line:
                col_num += 1
                if char == '0':
                    row.append(0)
                elif char == 'X':
                    row.append(1)
                elif char == 'S':
                    if (start_position is not None):
                        raise ValueError('Multiple start positions.')
                    start_position = (row_num, col_num)
                    row.append(0)
                elif char == 'G':
                    if (goal_position is not None):
                        raise ValueError('Multiple goal positions.')
                    goal_position = (row_num, col_num)
                    row.append(0)
            
            data.append(row)    

    if (start_position is None):
        raise ValueError('No start position.')
    if (goal_position is None):
        raise ValueError('No goal position.')
    
    return Map(np.array(data, dtype=np.int8), start_position, goal_position)

def write_map(map_obj, file_path):
    with open(file_path, 'w') as f:
        for y in range(0, map_obj.height, 1):
            for x in range(0, map_obj.width, 1):
                if (map_obj.is_start(y, x)):
                    f.write('S')
                elif (map_obj.is_goal(y, x)):
                    f.write('G')
                elif (map_obj.is_obstacle(y, x)):
                    f.write('X')
                elif (map_obj.is_clear(y,x)):
                    f.write('0')
                else:
                    f.write('?')
            f.write('\n')

def parse_args():
    parser = argparse.ArgumentParser(description = 'Run some path planning algorithms.')
    
    parser.add_argument('--input-file', required=True, dest='input_file', help='The input file to process.')
    parser.add_argument('--output-file', required=True, dest='output_file', help='The output file to process.')
    parser.add_argument('--algorithm-name', required=True, dest='algorithm_name', help='The algorithm name to run')

    args = parser.parse_args()
    return args

def run_algorithm(input_map_obj, algorithm_name):
    if (algorithm_name == 'echo'):
        return input_map_obj
    elif (algorithm_name == 'tangent_bug'):
        return run_tangent_bug(input_map_obj)
    else:
        raise ValueError('Unrecognized algorithm name: {0}'.format(algorithm_name))

def run_tangent_bug(input_map_obj):


def main():
    args = parse_args()
    
    print('Reading in map from {0}...'.format(args.input_file))
    map_obj = read_map(args.input_file)

    print('Running algorithm {0}...'.format(args.algorithm_name))
    map_obj_processed = run_algorithm(map_obj, args.algorithm_name)

    print('Writing map to {0}...'.format(args.output_file))
    write_map(map_obj_processed, args.output_file)

    print('Done.')

if __name__ == '__main__':
    main()
