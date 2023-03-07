#!/usr/bin/env python3
import os
import argparse
import random
import os.path
import sys

base_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(base_path, '..'))

COLORS = ["red", "green", "blue", "yellow"]

def main(args):
    with open('../../templates/random_cube_spawn_template.launch', "r") as template:
        filename = os.path.join(args.path, "{}.launch".format(args.filename))
        print("Writing file: ", filename)
        positions = set()

        with open(filename, "w") as templated_file:
                for line in template:
                    if "{{ADD_CUBES}}" in line:
                         for c in range(args.num_cubes):
                             color = random.choice(COLORS)
                             while True:
                                x, y = random.uniform(0, 3), random.uniform(0, 3)
                                if tuple((x,y)) not in positions:
                                    positions.add((x,y))
                                    break
                             templated_file.write('    <node name="spawn_{}_cube" pkg="gazebo_ros" type="spawn_model" args="-urdf -x {} -y {} -z 0.1 -param {}_cube_model -model {}_cube_{}" respawn="false" output="screen"/>\n'.format(c, x, y, color, color, c))
                    else:
                        templated_file.write(line)
                         
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, help="Launch file path", default="../../launch/")
    parser.add_argument('--filename', type=str, help="Launch file filename", default="random_cube_spawn")
    parser.add_argument('--num_cubes', type=int, help="Number of cubes to spawn", default=10)

    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    main(args)
