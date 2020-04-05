import os
import json
import colorama

CMAKE_OUTPUT_FNAME = 'compile_commands.json'

def get_build_directory():
    this_file_dir = os.path.abspath(os.path.dirname(__file__))
    up_one_directory = '/'.join(this_file_dir.split('/')[:-1])
    build_directory = os.path.join(up_one_directory, 'build')
    return build_directory

def read_cmake_compiler_commands_json(build_directory):
    compile_file = os.path.join(build_directory, CMAKE_OUTPUT_FNAME)
    if (not os.path.exists(compile_file)):
        print(colorama.Fore.RED, end='')
        print('**************************************************')
        print('ERROR: could not find compiler commands database.')
        print('Expected it to be at {0}'.format(compile_file))
        print('**************************************************')
        print(colorama.Style.RESET_ALL, end='')
        return None

    with open(compile_file, 'r') as f:
        return json.loads(f.read())

def write_json(folder_path, json_elements):
    if not (os.path.exists(folder_path)):
        os.makedirs(folder_path)

    output_path = os.path.join(folder_path, CMAKE_OUTPUT_FNAME)
    with open(output_path, 'w') as f:
        f.write(json.dumps(json_elements, indent=4, sort_keys=True))

def main():
    print('Regenerating YCM database...')

    build_directory = get_build_directory()
    all_compiler_commands = read_cmake_compiler_commands_json(build_directory)

    if (all_compiler_commands is None):
        return None

    partitioned = {}
    for compiler_command in all_compiler_commands:
        folder_name = compiler_command['directory'].split('/')[-1]
        if (folder_name not in partitioned):
            partitioned[folder_name] = []
        partitioned[folder_name].append(compiler_command)

    for folder_name in partitioned:
        write_json(os.path.join(build_directory, folder_name), partitioned[folder_name])

    # Leaving the default compiler commands file will generate lots of noise.
    os.remove(os.path.join(build_directory, CMAKE_OUTPUT_FNAME))

    print('YCM database regenerated sucessfully.')

if __name__ == '__main__':
    main()
