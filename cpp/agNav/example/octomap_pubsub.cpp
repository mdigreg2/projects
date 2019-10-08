#include<ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap_msgs/GetOctomap.h>

using namespace std;
using namespace octomap;

//callabck function to pull in an octomap reference from /octomap_full
void octoCallback(const octomap_msgs::Octomap &msg){	

  // The method fullMsgToMap takes in a pointer and returns an AbstractOcTree object pointer
  // AbstractOcTree is base class that all types of extended octrees inherit from
  AbstractOcTree *tree = octomap_msgs::fullMsgToMap(msg);

  //cast that abstract oc tree pointer to a normal OcTree object pointer
  OcTree *myOcTree = dynamic_cast<OcTree*>(tree);

  // Simple output to terminal counting nodes in tree 
  ROS_INFO("%zu Nodes Found in Tree", myOcTree->calcNumNodes()); 

  delete myOcTree;
}

int main(int argc, char **argv){

  //initialize a node named "height_map"
  ros::init(argc, argv, "octomap_pubsub");

  //generate a node handle
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/octomap_full", 1, octoCallback);

  // // Publisher structure shown here only as example, should use class for publisher in this case
  // ros::Publisher occ_pub = nh.advertise<sensor_msgs::PointCloud2>("octo_occupied", 1);
  // ros::Publisher free_pub = nh.advertise<sensor_msgs::PointCloud2>("octo_free", 1);
  
  ros::spin();
  
}
