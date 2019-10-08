#include<ros/ros.h>
#include<geometry_msgs/Point.h>
#include<visualization_msgs/MarkerArray.h>

#include<string>

#include<octomap_msgs/Octomap.h>
#include<octomap_msgs/conversions.h>
#include<octomap/octomap.h>
#include<octomap/OcTree.h>
#include<octomap/OcTreeNode.h>
#include<octomap/AbstractOcTree.h> 



class pillarTraversabilityMap{
public:

	//need a pointer to our ocTree

	octomap::OcTree *tree = NULL;

	//These will store point values for the max and min values returned from gteMetricMax() and getMetricMin()
	double xMin;
	double yMin;
	double zMin;
	double xMax;
	double yMax;
	double zMax;

	//These are going to be the points in question
	// octomap::point3d pointMin;
	// octomap::point3d pointMax;

	//create a node handler
	ros::NodeHandle nh;

	//a subscriber to subscribe to /octomap_full
	ros::Subscriber subOctomap;

	//a subsctriber to subscribe to a marker array if it is nexesarry
	ros::Subscriber subMarker;

	//a publisher to publish the coordinates of traversable regions 
	ros::Publisher pubOctomap;

	//constructor
	pillarTraversabilityMap():
	//private node handle
		nh("~"){

			//subctribe to /octomap_full, buffer of 1000, callback = octoCallBack()
			subOctomap = nh.subscribe("/octomap_full", 1000, &pillarTraversabilityMap::octoCallBack, this);

			//subscribe to /occupied_cells_vis_array, buffer of 1000, callback = markerCallBack()
			subMarker = nh.subscribe("/occupied_cells_vis_array", 1000, &pillarTraversabilityMap::markerCallBack, this);

			//publish to /pillar_traversability    +++++++++NOTE++++++++ should be switched to a series of coordinates
			pubOctomap = nh.advertise<octomap_msgs::Octomap>("/pillar_traversability", 1000);

		}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//callabck function handles pulling in new data
	//updating the min and max bounds
	//consistently updating functionality

	//takes in an octomap from /octomap_full or whatever was defined in the subscriber calling it
	void octoCallBack(const octomap_msgs::Octomap &msg){

		//AbstractOcTree* = points to the tree passed from the message
		octomap::AbstractOcTree *abTree = octomap_msgs::fullMsgToMap(msg);

		//tree now points to what was passed in by the callback
		tree = dynamic_cast<octomap::OcTree*>(abTree);
		//=====main functionality========================================================================================

		//get min and max bounds of the tree and store them in (xMin yMin zMin), (xMax yMax zMax)
		tree->getMetricMin(xMin, yMin, zMin);
		tree->getMetricMax(xMax, yMax, zMax);

		//make the min and max coordinates point3ds
		octomap::point3d pointMin(xMin, yMin, zMin);
		octomap::point3d pointMax(xMax, yMax, zMax);

		//get the resolution of the ocTree
		double res = tree->getResolution();

		//lastly create Octomap Keys for the coordinates of the min and max for easier reference and store them in minKey and maxKey
		octomap::OcTreeKey minKey = tree->coordToKey(pointMin, res);
		octomap::OcTreeKey maxKey = tree->coordToKey(pointMax, res);

		//===============================================================================================================
		//NEED TO PUBLISH WHAT IS REQUIRED BEFORE THIS
		//free the memory again
		delete abTree;

	}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//a callback function for any MarkerArrays that need to be taken in
	void markerCallBack(visualization_msgs::MarkerArray msg){
		ROS_INFO("NOT IMPLEMENTED");

	}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
		//This was intended to be the function to check the traversability status of each kernel, however, this may be implemented elsewhere
		void checkAtom(octomap::OcTreeKey &key, int abscissa, int ordinate, int height){
		
		//in case we are implementing this elsewhere, it is also not finished in its current state
		ROS_INFO("NOT IMPLEMENTED");

		key.k[0] = abscissa;
		key.k[1] = ordinate;
		key.k[2] = height;

	}

	
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// //calls the getMetricMin and Max functions at a given XYZ, references zMin and zMax should
// 	//either be initialized as whatever and then later changed by the functions, or equivalent
// 	//so that the function knows which voxel you're looking at 


// 	//Goes through bounding boxes given by ocTree Keys and then scans through to see if there
// 	//are gaps

// 	//gets the bounding box defined for a certain x, y coord
// 	//assumes that the key coords of the point are inline with origin in z field
// 	//additional function get Z should get the z coord of the robot
// 	void getBBX(float x, float y, float z){
// 		ROS_INFO("NOT IMPLEMENTED");
// 	}

// 	//What is needed
// 	/*
// 		1. create a grid around the robot the size of the map
// 		2. set a size for a "pillar"
// 		3. "cut" the grid up into regions corresponding to pillars
// 		4. iteratively search through these pillars for characteristics matching the "atomic unit" i.e
// 	 		an n x n x n+1 grid of free voxels where the bottom plane of voxels is occupied and the top n planes are free space
// 	*/
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 	//will store the current map corner bounds in the class variables x,y,z min and max
// 	void getMapBounds(pillarTraversabilityMap &obj){
// 		//octomap::getMetricMin(pillarTraversabilityMap::xMin, pillarTraversabilityMap::yMin, pillarTraversabilityMap::zMin);
// 		//octomap::getMetricMax(pillarTraversabilityMap::xMax, pillarTraversabilityMap::yMax, pillarTraversabilityMap::zMax);
// 		octomap::AbstractOcTree* abstractTree = dynamic_cast<octomap::AbstractOcTree*>(this->tree);
// 		abstractTree->getMetricMin(obj.xMin, obj.yMin, obj.zMin);
// 		abstractTree->getMetricMax(obj.xMax, obj.yMax, obj.zMax);
// 	}
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 	//will store the key values of the min and max bound
// 	void getMapBoundsKey(pillarTraversabilityMap *obj){
// 		obj->getMapBounds(*obj);
// 		const double xMinK = obj->xMin;
// 		const double yMinK = obj->yMin;
// 		const double zMinK = obj->zMin;
// 		const double xMaxK = obj->xMax;
// 		const double yMaxK = obj->yMax;
// 		const double zMaxK = obj->zMax;

// 		double res = obj->tree->getResolution();
// 		octomap::point3d pointMin(xMinK, yMinK, zMinK);
// 		octomap::point3d pointMax(xMaxKm yMaxK, zMaxK);
// 		const octomap::OcTreeKey minKey = obj->tree->coordToKey(pointMin, res);
// 		const octomap::OcTreeKey maxKey = obj->tree->coordToKey(pointMax, res);
// 	}

// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 	//This will go into the grid and then iterate through starting 
// 	//at the min and iterating through the x and then y, and then z
// 	//abscissa = x-direction, ordinate = y-direction, height = z-direction
// 	//giev it the location of the voxel that is on the first "free" layer above the single "occupied" layer which is at the bottom left of the atom 
// 	//when viewed from the top down with positive y pointing up and positve x pointing to the right

};


int main(int argc, char** argv){
	
	ros::init(argc, argv, "pillar_traversability");
	pillarTraversabilityMap p_trav_map;
	ros::spin();

	return 0;
} //int main()