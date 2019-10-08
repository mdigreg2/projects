#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeNode.h>
#include <TraversabilityOcTree.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <math.h>

class OccupiedPublisher{
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    float gs;
    
152
675
};