// #include "utility.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

// #include <stdlib.h>
// #include <stdio.h>
// #include <unistd.h>
// #include <math.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeNode.h>
#include <TraversabilityOcTree.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <octomap/AbstractOccupancyOcTree.h>
// #include <std_msgs/Float32.h>


class PlanarMap{

public:

    ros::NodeHandle nh;

    ros::Subscriber subOctree;
    ros::Publisher pubMap;

    std::vector<float> altitudes;
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    double idx, idy;

    double layer;

    nav_msgs::OccupancyGrid map;
    


    PlanarMap():
        nh("~"){

        pubMap = nh.advertise<nav_msgs::OccupancyGrid>("/planar_map", 5);
        subOctree = nh.subscribe<octomap_msgs::Octomap>("/octomap_full", 5, &PlanarMap::octomapCallback, this);

        map.header.frame_id = "map";
        map.info.origin.position.z = 2.0;

	nh.param("layer", layer, 0.0);
    }

    void octomapCallback(octomap_msgs::Octomap msg){
        octomap::TraversabilityOcTree *octTree = new octomap::TraversabilityOcTree(0.1);
        octomap::AbstractOcTree *abTree = octomap_msgs::fullMsgToMap(msg);
        octTree = dynamic_cast<octomap::TraversabilityOcTree*>(abTree);
        double res = octTree->getResolution();

        octTree->getMetricMin(minX, minY, minZ);
        octTree->getMetricMax(maxX, maxY, maxZ);

        std::cout << minX << minY << minZ << std::endl;

        map.info.resolution = res;
        map.info.width = maxX - minX +1;
        map.info.height = maxY - minY;
        map.info.origin.position.x = minX;
        map.info.origin.position.y = minY;
        map.data.clear();
        map.data.resize(map.info.width * map.info.height, -1);
        double ido = -minY*map.info.width - minX;	//id of the origin in the vector map.data

        for(octomap::TraversabilityOcTree::leaf_iterator it = octTree->begin_leafs(),end=octTree->end_leafs(); it!= end; ++it){
            //float desire = -2.0*res-res/2.0; //desired layer : just change the first value, here we want the -2 level
	  float desire = -layer*res-res/2.0; 
	  if (it.getZ() == desire){
            	if(it->getValue() > 0){
            		idx = it.getX()-res/2.0;
            		idy = (it.getY()-res/2.0)*map.info.width;
                	map.data[ido + idx + idy] = 100;
            	}
            	else{
            		idx = it.getX()-res/2.0;
            		idy = (it.getY()-res/2.0)*map.info.width;
                	map.data[ido + idx + idy] = 0;
            	}
           }
        }
        pubMap.publish(map);
        delete abTree;
        
    }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "PlanarMap");
    
    PlanarMap test_map;

    ros::spin();

    

    return 0;
}
