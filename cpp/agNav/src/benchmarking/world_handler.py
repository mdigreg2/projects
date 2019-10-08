import rospy
import rospkg
import roslaunch
import xml.etree.ElementTree as ET
import re
import os
import sys


# Custom class for world file error passing
class WorldError(Exception):
    pass


# Expands world file name to full path
def get_world_file(world_name):
    # Catch '$(find {package})' syntax
    match = re.search('^\s?(\$\(find\s+([\w\-]+)\))?(\/[\w\/\-\.]+)?([\w\/\-\.]+)?\s?$', world_name)

    if match is None:
        raise WorldError('Invalid "world_name" parameter, malformed string: ' + world_name)

    # Catch filename of format $(find {package})/...
    elif match.group(2) is not None and match.group(3) is not None:
        package = match.group(2)
        subdir = match.group(3)

        rospack = rospkg.RosPack() 
        
        try:
            pkgdir = rospack.get_path(package)
        except rospkg.common.ResourceNotFound as err:
            raise rospkg.common.ResourceNotFound('Ros package name not found: ' + package)
        
        filepath = pkgdir + subdir

    # Catch global path filename
    elif match.group(3) is not None:
        filepath = match.group(3)

    # Catch local filename
    elif match.group(4) is not None:
        filepath = os.getcwd() + '/' + match.group(4)

    # Catch edge case invalid syntax
    else:
        raise WorldError('Invalid "world_name" parameter, malformed string:', world_name)

    return filepath


# Returns 6DOF goal pose as list of floats
def find_goal_position(world_file_path, goal_name):
    tree = ET.parse(world_file_path)
    root = tree.getroot()
    if root is None:
        rospy.logerror('Malformed world file, no root')
        return

    # Attempt to get model pose from world file under state tag
    pose = root.findall('./world/state/model[@name="'+goal_name+'"]/pose')

    if pose == []:
        if root.findall('./world/state/model[@name="'+goal_name+'"]') == []:
            if root.findall('./world/model[@name="'+goal_name+'"]') != []:
                raise WorldError('Model definition "{}" exists, state pose missing'.format(goal_name))
            else:
                # If file is well-formed but goal is not initialized, return None
                return None
        else:
            if root.findall('./world/model[@name="'+goal_name+'"]') != []:
                raise WorldError(
                    'Model "{}" not listed in state, model definition and state exist'.format(goal_name))
            else:
                raise WorldError(
                    'Model definition "{}" does not exist while state does'.format(goal_name))

    # Ensure that pose (from findall) is returned as list even with one element
    if len(pose) > 1:
        raise WorldError('Multiple state definitions for model "{}"'.format(goal_name))

    pose_list = list(map(float, pose[0].text.split()))
    return pose_list


def get_world_file_list(world_names):
    if not isinstance(world_names, list):
        paths = [paths]
    world_files = []
    for name in world_names:
        world_files.append(get_world_file(name))
    return world_files


def get_goal_position_list(paths, goal_name):
    if not isinstance(paths, list):
        paths = [paths]
    goal_positions = []
    for path in paths:
        try:
            pose = find_goal_position(path, goal_name)
            goal_positions.append(pose)
        # If any world files have errors append their errors as strings
        # and continue as normal
        except WorldError as err:
            goal_positions.append(err)
        except IOError as err:
            goal_positions.append(err)
    return goal_positions


# Pretty print files and corresponding poses
def pprint_poses(paths, poses):

    col_s = [9, 60, 40]
    col_string = '{:>'+str(col_s[0])+'} | {:<'+str(col_s[1])+'} | {:<'+str(col_s[2])+'}'
    h_line_string = '{:-<'+str(sum(col_s))+'}'

    rospy.loginfo(col_string.format('Validity', 'File Name', 'Pose'))
    rospy.loginfo(h_line_string.format(''))

    for path, pose in zip(paths, poses):
        if pose is None:
            rospy.loginfo(col_string.format('Invalid:', path, pose))
        elif type(pose) is str:
            rospy.loginfo(col_string.format('Error:', path, pose))
        else:
            rospy.loginfo(col_string.format('Valid:', path, pose))


# Open gazebo world and return pose of goal object after closing
def open_world_edit(path):
    rospy.loginfo('Press Enter once done saving with Gazebo GUI...')

    # Save current sys.stdout in a variable and replace with /dev/null
    # Would be more elegant hide output from launch file directly, but
    # force_log api not implemented in ROS Kinetic
    stdout_save = sys.stdout
    sys.stdout = open(os.devnull, 'w')

    # TODO: fix includes in module and scripts
    # Initialization of launch object, used to launch nodes that will restart with each episode
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Set world_name parameter to be hackishly interpreted as an argument in launch file
    rospy.set_param('/world_name', path)

    # Configure roslaunch and execute
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [sys.path[0]+'/../launch/include/validator/config_world.launch.xml']) # TODO: make launch file path passable
    launch.start()

    # Must resume stdout here for clipboard support to work properly
    sys.stdout = stdout_save

    # Use raw_input() to block until done with gazebo
    raw_input()

    # Resume dumping of stdout to hide roslaunch output
    sys.stdout = open(os.devnull, 'w')

    # Explicitly kill gzserver and gzclient to save time waiting for sigterm, then shutdown
    os.system("killall -9 gzserver")
    os.system("killall -9 gzclient")
    launch.shutdown()

    # Restore stdout from variable second time
    sys.stdout = stdout_save

    return find_goal_position(path, 'goal_position')
    
