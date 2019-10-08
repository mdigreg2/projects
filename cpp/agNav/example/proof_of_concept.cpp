////////////////////////////////////////////////////////////////////////////////////////////////////
////	Includes
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "utility.h"


using namespace std;

// varaibles acquises
double hauteur;
double abscisse;
double ordonnees;
std::vector<float> altitudes;


////////////////////////////////////////////////////////////////////////////////////////////////////
////	Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void chatterCallback(const visualization_msgs::MarkerArray msg)
{
	for (unsigned int i = 0 ; i < msg.markers[15].points.size(); i++){
		hauteur = msg.markers[15].points[i].z;
		altitudes.push_back(hauteur);
	}
		
}

void print(std::vector<float> const &input)
{
	for (int i = 0; i < input.size(); i++) {
		std::cout << input.at(i) << ' ';
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////	Main
////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){

    ros::init(argc, argv, "height_map");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber subMarker = nh.subscribe("occupied_cells_vis_array", 1000, chatterCallback);
    // Publishers
    //ros::Publisher chatter_pub = nh.advertise<std::vector<geometry_msgs::Point>>("send_point", 1000);

    ros::Rate loop_rate(25);

	while (ros::ok()){
		ros::spinOnce();
		print(altitudes);
        	//chatter_pub.publish(altitudes);

		loop_rate.sleep();
	}
	return 0;
}
