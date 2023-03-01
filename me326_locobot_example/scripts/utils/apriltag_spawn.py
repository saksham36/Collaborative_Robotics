#!/usr/bin/env python3
import os
import argparse
import random
import os.path
import sys

base_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(base_path, '..'))

APRILTAG_BASENAME = "Apriltag36_11_"

def main(args):
    with open('../templates/apriltag_template.launch', "r") as template:
        filename = os.path.join(args.path, "{}.launch".format(args.filename))
        print("Writing file: ", filename)
        positions = set()

        with open(filename, "w") as templated_file:
                for line in template:
                    if "{{ADD_APRILTAGS}}" in line:
                        for c in range(args.num_tags):
                            tag_name = APRILTAG_BASENAME + str(c).zfill(5)
                            while True:
                                x, y = random.uniform(1.7, 5), random.uniform(1.7, 5)
                                if tuple((x,y)) not in positions:
                                    positions.add((x,y))
                                    break

                            templated_file.write('    <param name="{}_model" textfile="$(find me326_locobot_example)/model/{}/model.sdf"/>\n'.format(tag_name, tag_name))
                            templated_file.write('    <node name="spawn_{}" pkg="gazebo_ros" type="spawn_model" args="-sdf -x {} -y {} -z 0.1 -param {}_model -model {}" respawn="false" output="screen"/>\n'.format(tag_name, x, y, tag_name, tag_name))
                    else:
                        templated_file.write(line)
                         
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, help="Launch file path", default="../launch/")
    parser.add_argument('--filename', type=str, help="Launch file filename", default="apriltag_spawn")
    parser.add_argument('--num_tags', type=int, help="Number of apriltags to spawn", default=3)

    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    main(args)
