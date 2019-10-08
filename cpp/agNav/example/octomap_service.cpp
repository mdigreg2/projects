#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap_msgs/GetOctomap.h>

using octomap_msgs::GetOctomap;
using namespace std;
using namespace octomap;

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_service");
  ros::NodeHandle nh;

  GetOctomap::Request req;
  GetOctomap::Response resp;
  std::string servname = "octomap_full";

  while(ros::ok() && !ros::service::call(servname, req, resp))
    {
      ROS_WARN("Request to %s failed; trying again...", nh.resolveName(servname).c_str());	    
      ros::Duration(0.5).sleep();
    }

  if (ros::ok()) {

    AbstractOcTree* tree = octomap_msgs::msgToMap(resp.map);
    OcTree* octree = NULL;

    if (tree)
      {
	octree = dynamic_cast<OcTree*>(tree);
      }
    else
      {
	ROS_ERROR("Error creating octree from received message");
      }

    if (octree)
      {
	// Simple output to terminal counting nodes in tree 
	ROS_INFO("%zu Nodes Found in Tree", octree->calcNumNodes()); 
      }
    else
      {
	ROS_ERROR("Error reading OcTree from stream");
      }
    
    delete octree;   
  }    
}
