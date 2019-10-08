// #include "utility.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeNode.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <octomap/AbstractOccupancyOcTree.h>
// #include <std_msgs/Float32.h>


class TraversabilityHeight{

public:

    ros::NodeHandle nh;

    ros::Subscriber subOctree;
    ros::Publisher pubHeight;

    float height;
    float abscisse;
    float ordonnees;
    int cmpt;
    sensor_msgs::PointCloud2 sortie;
    sensor_msgs::PointField first;
    sensor_msgs::PointField second;
    sensor_msgs::PointField third;

    TraversabilityHeight():
        nh("~"){

        pubHeight = nh.advertise<sensor_msgs::PointCloud2>("/test_height", 5);
        subOctree = nh.subscribe<octomap_msgs::Octomap>("/octomap_full", 5, &TraversabilityHeight::octomapCallback, this);

        sortie.header.frame_id = "map";
        sortie.height = 1;
        
        first.name = "x";
        first.offset = 0;
        first.datatype = 7;
        first.count = 1;

        second.name = "y";
        second.offset = 4;
        second.datatype = 7;
        second.count = 1;

        third.name = "z";
        third.offset = 8;
        third.datatype = 7;
        third.count = 1;

        sortie.fields.push_back(first);
        sortie.fields.push_back(second);
        sortie.fields.push_back(third);
        sortie.point_step = 12;
        sortie.is_bigendian = false;
        sortie.is_dense = true;
    }

    void octomapCallback(octomap_msgs::Octomap msg){
        octomap::OcTree *octTree = NULL;
        octomap::AbstractOcTree *abTree = octomap_msgs::fullMsgToMap(msg);
        octomap::OcTree *tempTree = octTree;
        octTree = dynamic_cast<octomap::OcTree*>(abTree);
        cmpt = 0;
        sortie.data.clear();    
        for(octomap::OcTree::leaf_iterator it = octTree->begin_leafs(),end=octTree->end_leafs(); it!= end; ++it){
            //manipulate node, e.g.:
            //std::cout << "Node center: " << it.getCoordinate() << std::endl;
            //std::cout << "Node size: " << it.getSize() << std::endl;
            //std::cout << "Node value: " << it->getValue() << std::endl;
            if(it->getValue() > 0){
                float x = it.getX();
                float y = it.getY();
                float z = it.getZ();
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&x)[0]);
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&x)[1]);
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&x)[2]);
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&x)[3]);
                
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&y)[0]);
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&y)[1]);
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&y)[2]);
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&y)[3]);
                
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&z)[0]);
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&z)[1]);
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&z)[2]);
                //sortie.data.push_back(reinterpret_cast<uint8_t*>(&z)[3]);

                sortie.data.insert(sortie.data.end(),reinterpret_cast<uint8_t*>(&x), reinterpret_cast<uint8_t*>(&x)+4);
                sortie.data.insert(sortie.data.end(),reinterpret_cast<uint8_t*>(&y), reinterpret_cast<uint8_t*>(&y)+4);
                sortie.data.insert(sortie.data.end(),reinterpret_cast<uint8_t*>(&z), reinterpret_cast<uint8_t*>(&z)+4);
                cmpt = cmpt +1;
            }
        }
        sortie.width = cmpt;
        sortie.row_step = sortie.width*3;
        pubHeight.publish(sortie);

        if (tempTree != NULL){
            delete tempTree;
        }
        delete abTree;

    }

};


int main(int argc, char** argv){

    ros::init(argc, argv, "traversability_height");
    
    TraversabilityHeight heightMap;

    ros::spin();

    

    return 0;
}
