#!/usr/bin/env python
import rospy
import roslaunch
import sys
import os
import world_handler as wh
import argparse

try:
    import pyperclip
    clipboard_support = True
except ImportError:
    print('Warning: "pyperclip" package not found, gazebo world file saving will be manual')
    clipboard_support = False

    
def yes_no_prompt(q):
    resp = raw_input(q).lower().strip()
    if resp == 'y':
        return True
    elif resp == 'n':
        return False
    else:
        yes_no_prompt(q)        


# This file serves as a verification step to ensure that the listed world files contain a valid goal position
# Further, helper functionality is included to launch gazebo if a particular world file is without a goal
def main():
    rospy.init_node('gazebo_world_validator')
    
    # Run argparse for command line interface
    parser = argparse.ArgumentParser(prog='gazebo_world_validator')
    parser.add_argument('world_name', nargs='+', help='file path of target world file')
    parser.add_argument('-p', '--only-print', action='store_true', help='print validity of world files and exit')
    parser.add_argument('-e', '--edit', action='store_true', help='edit world file even if pose is already available')
    args = parser.parse_args()

    # Set argument variables
    edit_flag = args.edit
    only_print = args.only_print
    world_names = args.world_name

    # TODO: Implement reading from rosparam if arg(s) not passed
    #       Issues present with named arguments passed when invoked from roslaunch
    #       Must be invoked directly from rosrun or command line until implemented
    # # Pull from ros parameter 'world_name' if argument not passed
    # if world_names == []:
    #     if rospy.has_param('world_param'):
    #         world_names = rospy.get_param(rospy.get_param('world_param'))
    #     else:
    #         print('Error: no world file given in arguments or param')
    #         return

    world_paths = wh.get_world_file_list(world_names)
    world_poses = wh.get_goal_position_list(world_paths, 'goal_position')

    wh.pprint_poses(world_names, world_poses)
    
    if only_print == True:
        return
    
    for path, pose, name in zip(world_paths, world_poses, world_names):
        if edit_flag == True:
            pose = None
        if pose is None:
            if yes_no_prompt('Open Gazebo to edit world file "{}"? [y/n]'.format(name)):
                while pose is None:
                    if clipboard_support:
                        pyperclip.copy(path)
                        print('"{}" copied to clipboard, paste in Gazebo "Save As" prompt'.format(path))

                    pose = wh.open_world_edit(path)
                    if pose is None:
                        if not yes_no_prompt('Pose still not initialized, try again? [y/n]'):
                            print('File "{}" ignored'.format(path))
                            break
                    else:
                        print('File "{}" successfully stored goal pose {}'.format(name, pose))

            else:
                print('File "{}" ignored'.format(path))

    
if __name__ == '__main__':
    main()
