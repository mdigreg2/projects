////////////////////////////////////////////////////////////////////////////////////////////////////
////	Includes
////////////////////////////////////////////////////////////////////////////////////////////////////


//pulled ths from tixiao's repo but I don't think that we need all of this so I'm going to comment it   --mike
//#include "utility.h"
#include<ros/ros.h>
#include"std_msgs/Float64.h"
#include<visualization_msgs/MarkerArray.h>
#include<geometry_msgs/Point.h>
#include<cstdlib>

#include<octomap/octomap.h>
#include<octomap/OcTree.h>

using namespace std;

//declare the vector that we will be storing height data in
vector<float> altitudes;

//declare a float we will be using to store each individual point's height
float height;

//make a geo point named voxel
geometry_msgs::Point voxel;

////////////////////////////////////////////////////////////////////////////////////////////////////
////	Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

//callback function for pulling occupied cell coordinates from a message of type visualization_msgs/MarkerArray
void pointsCallback(const visualization_msgs::MarkerArray msg)
{	
	//iterate through all of the points in markers[15]
	for(unsigned int i = 0; i < msg.markers[15].points.size(); i++){

		//set the height equal to the z value of the point in question
		height = msg.markers[15].points[i].z;

		//set the voxel's z equal to the z of the point in question (useful if we need to make a cloud)
		voxel.z = height;

		//print the height that we are pushing to the vector for debugging purposes
		//ROS_INFO("I'm pushing: [%f]", height);

		//push the heights onto the vector
		altitudes.push_back(height);

	}




		
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////	Main
////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){

	//initialize a node named "height_map"
    ros::init(argc, argv, "altitude_map");

    //generate a node handle
    ros::NodeHandle nh;

    //Subscribe to occupied_cells_vis_array (published by octomap_server at the time of writing)
    //store 1000 values in buffer before dropping the oldest ones
    //use the function pointsCallback to pull more data from the topic
    ros::Subscriber sub = nh.subscribe("occupied_cells_vis_array", 1000, pointsCallback);

    //Publish to occupied_altitudes
    //publish the heights as a Float64 
    //buffer queue of 1000 before we start dumping old messages that havent been sent
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("occupied_altitudes", 1000);

    //send messages at 25Hz, however occupied_cells_vis_array seems to publish at 8Hz
    ros::Rate loop_rate(25);

	while (ros::ok()){

		//variable to hold onto our z coordinates
		std_msgs::Float64 z_coord;

		//Iterate through the altitudes vector structure and publish the height values 
		//from beginning to end
		for(vector<float>::iterator i = altitudes.begin(); i != altitudes.end(); i++){
			float altitude = *i;
			z_coord.data = altitude;
			pub.publish(z_coord);
		}

		//loop and call our callback function for the subscriber so that we can keep updating values
		ros::spinOnce();

		//sleep so that we can keep up with our publisher frequency of 25hz
		loop_rate.sleep();
	} //while(ros::ok())

	return 0;
}  //int main
