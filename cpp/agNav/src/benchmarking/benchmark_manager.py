#!/usr/bin/env python
"""
Manager node for navigation benchmarking
"""

import rospy
import roslaunch
import tf
import world_handler as wh
from gazebo_msgs.srv import GetLinkState
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_srvs.srv import Empty

import os
import time
import math

class BenchmarkRunData():

    def __init__(self):
        self.dist_tf = 0
        self.dist_gazebo = 0
        self.wheel_spin = [0, 0, 0, 0]

        self.total_tf_pos = [0, 0, 0]
        self.total_gazebo_pos = [0, 0, 0]
        
        self.cur_tf_pos = None
        self.cur_gazebo_pos = None
        self.cur_wheel_pos = None

        self.real_start_time = time.time()
        self.real_end_time = None
        self.real_duration = None

        self.ros_start_time = rospy.Time.now()
        self.ros_end_time = None
        self.ros_duration = None
        
    # def start(self):
    #     self.real_start_time = time.time()
    #     self.ros_start_time = rospy.Time.now() 

    def end(self):
        self.real_end_time = time.time()
        self.ros_end_time = rospy.Time.now()
        self.real_duration = self.real_end_time - self.real_start_time
        self.ros_duration = self.ros_end_time - self.ros_start_time

        self.dist_tf = math.sqrt(sum([x**2 for x in self.total_tf_pos]))
        self.dist_gazebo = math.sqrt(sum([x**2 for x in self.total_gazebo_pos]))

    def output(self):
        return [self.dist_tf] + [self.dist_gazebo] + self.wheel_spin
    
    def update_tf_dist(self, translation, rotation=None):
        if self.cur_tf_pos is not None:
            self.total_tf_pos = [c + abs(a - b) for a, b, c in zip(translation,
                                                                   self.cur_tf_pos,
                                                                   self.total_tf_pos)]
        self.cur_tf_pos = translation

    def update_gazebo_dist(self, translation, rotation=None):
        if self.cur_gazebo_pos is not None:
            self.total_gazebo_pos = [c + abs(a - b) for a, b, c in zip(translation,
                                                                       self.cur_gazebo_pos,
                                                                       self.total_gazebo_pos)]
        self.cur_gazebo_pos = translation

    def update_wheel_spin(self, f_right, f_left, r_right, r_left):
        if cur_wheel_pos is not None:
            pass
            # TODO: Need to determine simple method to add changes
            # in quaternions or euler angles in target frame
        else:
            cur_wheel_pos = [f_right, f_left, r_right, r_left]
        
class BenchmarkManager():

    def __init__(self, worlds, traversabilities, path_planners, robot_stack):
        self.run_list = [{'world':w, 'trav':t, 'path':p}
                         for w in worlds
                         for t in traversabilities
                         for p in path_planners]
        self.run_iter = iter(self.run_list)
        
        self.world_launch = None
        self.trav_launch = None
        self.path_launch = None
        self.stack_launch = self.configure_launch(robot_stack)

        self.cur_world = None
        self.move_base_goal_flag = False
        
        self.listener = tf.TransformListener()
        self.goal_publisher = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10, latch=True)
        self.result_subscriber = rospy.Subscriber(
	    '/move_base/result', MoveBaseActionResult, self.move_base_goal_reached)

        self.data = []


    def get_tf_update(self, frame):
        """Return current tf from map to 'frame'"""
        self.listener.waitForTransform("map", frame, rospy.Time(), rospy.Duration(20.0))
        
        now = rospy.Time.now()
        self.listener.waitForTransform("map", frame, now, rospy.Duration(5.0))
	(tf_trans,tf_orient) = self.listener.lookupTransform("map", frame, now)
        
        return (tf_trans, tf_orient)

    def get_gazebo_update(self, link, ref=''):
        """Return pose of gazebo object 'name'"""
        rospy.wait_for_service('/gazebo/get_link_state')
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        link = get_link_state(link, ref)
        translation = [link.link_state.pose.position.x,
                       link.link_state.pose.position.y,
                       link.link_state.pose.position.z]
        rotation  = [link.link_state.pose.orientation.x,
                     link.link_state.pose.orientation.y,
                     link.link_state.pose.orientation.z,
                     link.link_state.pose.orientation.w]
        return (translation, rotation)

    def restart_gazebo_world(self, world):
        """Restart gazebo world"""
        # # Below is gazebo service reset for cases where same world is reused
        # # Removed as service reset causes tf buffer clear issues
        # if self.cur_world == world:
        #     rospy.wait_for_service('/gazebo/reset_simulation')
        #     reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        #     reset_simulation()
        #     rospy.logwarn('Invoked /gazebo/reset_simulation service call')
        # else:
        if self.cur_world is not None:
            os.system("killall -9 gzserver")
            os.system("killall -9 gzclient")            
            self.world_launch.shutdown()
            rospy.logwarn('Stopped gazebo world {}'.format(self.cur_world))

        # Set world_name parameter to be hackishly interpreted as an argument in launch file
        rospy.set_param('/world_name', wh.get_world_file(world))

        self.world_launch = self.configure_launch(wh.get_world_file(
            '$(find aggressive-navigation)/launch/include/gazebo/jackal_world.launch'))
        self.world_launch.start()

        self.cur_world = world
        rospy.logwarn('Started new gazebo world {}'.format(self.cur_world))

    def restart_traversability(self, trav):
        """Restart traversability launch file"""
        try:
            self.trav_launch.shutdown()
            rospy.logwarn('Stopped traversability node {}'.format(self.trav_launch.roslaunch_files))
        except AttributeError:
            pass
        self.trav_launch = self.configure_launch(wh.get_world_file(trav))
        self.trav_launch.start()
        rospy.logwarn('Started traversability node {}'.format(self.trav_launch.roslaunch_files))
                          
    def restart_path_planner(self, path):
        """Restart path planner launch file"""
        try:
            self.path_launch.shutdown()
            rospy.logwarn('Stopped path planner node {}'.format(self.path_launch.roslaunch_files))
        except AttributeError:
            pass
        self.path_launch = self.configure_launch(wh.get_world_file(path))
        self.path_launch.start()
        rospy.logwarn('Started path planner node {}'.format(self.path_launch.roslaunch_files))

    def restart_robot_stack(self):
        """Restart launch file for basic robot stack (LOAM, etc.)"""
        self.stack_launch.shutdown()
        rospy.logwarn('Stopped robot stack {}'.format(self.stack_launch.roslaunch_files))

        self.stack_launch.start()
        rospy.logwarn('Started robot stack {}'.format(self.stack_launch.roslaunch_files))
        
    def configure_launch(self, launch_file):
        """Return roslaunch object from passed launch file path string"""
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Configure roslaunch and execute, saving object to self.world_launch
        launch = roslaunch.parent.ROSLaunchParent(
            uuid,
            [launch_file])
        return launch

    def publish_goal(self):
        """Publishes 'pose' argument as goal point in map frame for path planner"""

        # Hacky solution to block until move_base and gazebo are up
        rospy.wait_for_service('/move_base/set_parameters')
        rospy.wait_for_service('/gazebo/get_link_state')

        # Make gazebo service call for tf from jackal base_link to goal
        (gz_trans, gz_rot) = self.get_gazebo_update('goal_position::link', 'jackal::base_link')

        # Lookup baselink transform until available
        (tf_trans, tf_rot) = self.get_tf_update('base_link')

        full_trans = [a + b for a, b in zip(gz_trans, tf_trans)]
        full_rot = [a + b for a, b in zip(gz_rot, tf_rot)]

        goal = PoseStamped()
        
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.position.x = full_trans[0]
        goal.pose.position.y = full_trans[1]
        goal.pose.position.z = full_trans[2]
        goal.pose.orientation.x = full_rot[0]
        goal.pose.orientation.y = full_rot[1]
        goal.pose.orientation.z = full_rot[2]
        goal.pose.orientation.w = full_rot[3]
        
        self.goal_publisher.publish(goal)
        rospy.logwarn('Published move_base goal at position {}'.format(full_trans))

    def move_base_goal_reached(self, data):
        if data.status.text == "Goal reached.":
            self.move_base_goal_flag = True
            rospy.logwarn('Goal position reached')
        return
    
    def log_until_move_base(self, max_duration=None):
        """Logs run data until goal is reached according to move_base"""
        r = rospy.Rate(10)
        self.cur_data_log = BenchmarkRunData()
        while self.move_base_goal_flag == False and not rospy.is_shutdown():
            (tf_trans, tf_rot) = self.get_tf_update('base_link')
            (gz_trans, gz_rot) = self.get_gazebo_update('jackal::base_link')
            self.cur_data_log.update_tf_dist(tf_trans, tf_rot)
            self.cur_data_log.update_gazebo_dist(gz_trans, gz_rot)
            r.sleep()
        self.cur_data_log.end()
        self.data.append(self.cur_data_log.output())
        
        self.move_base_goal_flag = False

        rospy.logwarn('Run finished, results logged')
        

    def log_until_gazebo(self, max_duration=None):
        """Logs run data until goal is reached according to gazebo"""
        raise NotImplementedError

    def push_current_log(self):
        """Add data logged from last run into multi-run list"""
        raise NotImplementedError
    
    def run_next(self):
        """Iterate once over run_params and start run with next data set"""
        run_params = self.run_iter.next()

        # try:
        #     self.path_launch.shutdown()
        #     self.trav_launch.shutdown()
        #     self.stack_launch.shutdown()
        # except AttributeError as err:
        #     rospy.logwarn(err)
            
        self.restart_gazebo_world(run_params.get('world'))

        self.listener._listener.buffer.clear()
        self.listener = tf.TransformListener()
        
        self.restart_robot_stack()
        self.restart_traversability(run_params.get('trav'))
        self.restart_path_planner(run_params.get('path'))

        self.publish_goal()

        self.log_until_move_base()
        # self.push_current_log()

    def write_csv(self, filename):
        """Write full data output in csv format to 'filename'"""
        raise NotImplementedError

    def print_all(self):
        """Print full data output to console"""
        raise NotImplementedError

    def print_cur(self):
        """Print current run values to console"""
        raise NotImplementedError

# TODO: write gazebo_benchmark function in module
def main():
    rospy.init_node("benchmark_manager", anonymous=True)

    # # TODO: Perhaps should use this structure for parameter, look over carla param usage
    # params = rospy.get_param('carla')
    # host = params['host']
    # port = params['port']
    # num_episodes = params['Episodes']
    # frame_max = params['FramesTotal']
    # current_frame = 0

    world_param = rospy.get_param('world_param')
    trav_param = rospy.get_param('trav_param')
    path_param = rospy.get_param('path_param')

    world_files = rospy.get_param(world_param)
    traversability_files = rospy.get_param(trav_param)
    path_planner_files = rospy.get_param(path_param)
    stack_file = rospy.get_param('stack_launch')

    bm = BenchmarkManager(world_files,
                          traversability_files,
                          path_planner_files,
                          stack_file)

    while(1):
        try:
            bm.run_next()
            rospy.logwarn('Run results: {}'.format(bm.cur_data_log.output()))
        except StopIteration:
            break
            
    rospy.logwarn(bm.data)
        

if __name__ == "__main__":
    main()
