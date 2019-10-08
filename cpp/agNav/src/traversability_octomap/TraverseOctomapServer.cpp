/*
 * Copyright (c) 2012, D. Kuhner, P. Ruchti, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <TraverseOctomapServer.h>
#include <string>

using namespace octomap;

namespace octomap_server {

  TraverseOctomapServer::TraverseOctomapServer(const std::string& filename) :
    OctomapServer()
  {
    //read tree if necessary
    if (filename != "") {
      if (m_octree->readBinary(filename)) {
	ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), m_octree->size());
	m_treeDepth = m_octree->getTreeDepth();
	m_res = m_octree->getResolution();
	m_gridmap.info.resolution = m_res;

	publishAll();
      } else {
	ROS_ERROR("Could not open requested file %s, exiting.", filename.c_str());
	exit(-1);
      }
    }

    ros::NodeHandle private_nh("~");

    std::string changeSetTopic = "changes";
    std::string changeSetTopicKernel = "/octo_change_kernel";
    std::string changeIdFrame = "/talker/changes";

    private_nh.param("topic_changes", changeSetTopic, changeSetTopic);
    private_nh.param("topic_changes_kernel", changeSetTopicKernel, changeSetTopicKernel);
    private_nh.param("change_id_frame", change_id_frame, changeIdFrame);
    private_nh.param("track_changes", track_changes, false);
    private_nh.param("listen_changes", listen_changes, false);
    private_nh.param("min_change_pub", min_change_pub, 0);

    private_nh.param("robot_length", robot_length, 0.5);
    private_nh.param("robot_width", robot_width, 0.5);
    private_nh.param("robot_height", robot_height, 0.5);

    assert(robot_length > 0);
    assert(robot_width > 0);
    assert(robot_height > 0);

    // Number of voxels in each dimension of kernel
    kernel_dim[0] = ceil(robot_length / m_octree->getResolution());
    kernel_dim[1] = ceil(robot_width / m_octree->getResolution());
    kernel_dim[2] = ceil(robot_height / m_octree->getResolution()) + 1; // +1 here for additional height voxel row

    // Size of kernel in total voxels
    kernel_size = kernel_dim[0] * kernel_dim[1] * kernel_dim[2];
    
    // Kernel offset is kernel dimensions in meters minus root voxel
    kernel_offset = point3d((kernel_dim[0] - 1) * m_octree->getResolution(),
			    (kernel_dim[1] - 1) * m_octree->getResolution(),
			    (kernel_dim[2] - 1) * m_octree->getResolution());
    
    // Dynamically allocate a 3D comparison kernel
    // Below is basic, flat ground kernel init
    kernel = new float **[kernel_dim[0]]();
    // Iterate in x direction
    for (int i = 0; i < kernel_dim[0]; i++)
      {
	// Iterate in y direction
	
        kernel[i] = new float *[kernel_dim[1]]();
        for (int j = 0; j < kernel_dim[1]; j++)
	  {
	    // Iterate in z direcion
	    kernel[i][j] = new float [kernel_dim[2]]();
	    // First value of z is max occupied, all others are max free
	    // TODO: compare against probability instead of log-odds?
	    for (int k = 0; k < kernel_dim[2]; k++)
	      {
		kernel[i][j][k] = 2.6;
	      }
	    kernel[i][j][0] = -2.6;
	  }
      }

    ROS_WARN("Kernel dims: %i, %i, %i", kernel_dim[0], kernel_dim[1], kernel_dim[2]);

    if (track_changes && listen_changes) {
      ROS_ERROR("OctoMapServer: It might not be useful to publish changes and at the same time listen to them."
		"Setting 'track_changes' to false!");
      track_changes = false;
    }

    if (track_changes) {
      ROS_INFO("starting server");
      pubChangeSet = private_nh.advertise<sensor_msgs::PointCloud2>(changeSetTopic, 1);
      pubChangeSetKernel = private_nh.advertise<sensor_msgs::PointCloud2>(changeSetTopicKernel, 1);
      m_octree->enableChangeDetection(true);
    }

    if (listen_changes) {
      ROS_INFO("starting client");
      subChangeSet = private_nh.subscribe(changeSetTopic, 1,
					  &TraverseOctomapServer::trackCallback, this);
    }

  }

  TraverseOctomapServer::~TraverseOctomapServer() {
    //Dynamically deallocate a 3D kernel array
    for (int i = 0; i < kernel_dim[0]; i++)
      {
	for (int j = 0; j < kernel_dim[1]; j++)
	  delete[] kernel[i][j];
	delete[] kernel[i];
      }
    delete[] kernel;
  }

  int TraverseOctomapServer::getIterMin(const float& center, const float& offset, const float& root_center) {
    float min_ind = center - offset;
    if (min_ind < 0)
      min_ind = 0;
    min_ind -= root_center;
    return (int)roundf(min_ind);
  }
  
  int TraverseOctomapServer::getIterMax(const float& center, const float& offset, const float& root_center, const int& k_max) {
    float max_ind = center + offset;
    if (max_ind > k_max)
      max_ind = k_max;
    max_ind -= root_center;
    return (int)roundf(max_ind);
  }

  // Function to perform score update behavior
  // Set point cloud for now but will set octree in future
  void TraverseOctomapServer::updateNodeScore(const float& score, const OcTreeNode& node){
    
  }

  float TraverseOctomapServer::calculateTraversability(const point3d& center) {

    float score = 0;
    int voxels_hit = 0;
	
    for (OcTreeT::leaf_bbx_iterator kernel_it = m_octree->begin_leafs_bbx(center,
									  center + kernel_offset,
									  m_treeDepth),
	   kernel_end = m_octree->end_leafs_bbx(); kernel_it != kernel_end; ++kernel_it)
      {
	point3d kernel_center = kernel_it.getCoordinate();
	point3d ind = kernel_it.getCoordinate() - center;
	float value = kernel_it->getValue();
	uint8_t depth = kernel_it.getDepth();

	if (depth < m_octree->getTreeDepth())
	  {
	    float offset = getNodeOffset(depth);
	    int iter_start[3];
	    int iter_end[3];
		  
	    for (int n = 0; n < 3; n++)
	      {
		iter_start[n] = getIterMin(kernel_center(n), offset, center(n));
		iter_end[n] = getIterMax(kernel_center(n), offset, center(n), kernel_dim[n]);
	      }
		      
	    for (int i = iter_start[0]; i < iter_end[0]; i++)
	      {
		for (int j = iter_end[1]; j < iter_end[1]; j++)
		  {
		    for (int k = iter_start[2]; k < iter_end[2]; k++)
		      {
			score += std::abs(kernel[i][j][k] - value);
			voxels_hit++;
		      }
		  }
	      }

	  }
	      
	else
	  {
	    // // TODO: Fix conditional to handle rounded value
	    // if ((int)roundf(ind.x()/m_octree->getResolution()) < 0 ||
	    // 	(int)roundf(ind.x()/m_octree->getResolution()) >= kernel_dim[0] ||
	    // 	(int)roundf(ind.y()/m_octree->getResolution()) < 0 ||
	    // 	(int)roundf(ind.y()/m_octree->getResolution()) >= kernel_dim[1] ||
	    // 	(int)roundf(ind.z()/m_octree->getResolution()) < 0 ||
	    // 	(int)roundf(ind.z()/m_octree->getResolution()) >= kernel_dim[2])
	    //   {
	    // 	ROS_WARN("Depth: %i", depth);
	    // 	ROS_WARN("Root Coord: %f, %f, %f", center.x(), center.y(), center.z());
	    // 	ROS_WARN("Cur Coord = %f, %f, %f", kernel_center.x(), kernel_center.y(), kernel_center.z());
	    // 	ROS_WARN("Voxel val = %f", value);
	    // 	ROS_WARN("Float Indices = %f, %f, %f", (ind.x()), (ind.y()), (ind.z()));
	    // 	ROS_WARN("Int Indices = %i, %i, %i", (int)roundf(ind.x()/m_octree->getResolution()), (int)roundf(ind.y()/m_octree->getResolution()), (int)roundf(ind.z()/m_octree->getResolution()));
	    // 	m_octree->resetChangeDetection();
	    // 	return 0;
	    //   }
	    // Perform kernel comparison
	    score += std::abs(kernel
			      [ (int)roundf(ind.x()/m_octree->getResolution()) ]
			      [ (int)roundf(ind.y()/m_octree->getResolution()) ]
			      [ (int)roundf(ind.z()/m_octree->getResolution()) ] - value);
	    // Increment count of voxels hit
	    voxels_hit++;
	  }
      }
    // Add uncertain values that were never iterated
    score += (kernel_size - voxels_hit) * 0.5;
    return score;
  }
  
  void TraverseOctomapServer::exhaustiveKernel(const point3d& center) {
    for (TraversabilityOcTree::leaf_bbx_iterator root_iter = m_octree->begin_leafs_bbx(center - kernel_offset,
									 center,
									 m_treeDepth),
	   root_end = m_octree->end_leafs_bbx(); root_iter != root_end; ++root_iter)
      {
	// Initialize score values
	int voxels_hit = 0;
	float score = 0.0;
	  
	point3d root_center = root_iter.getCoordinate();	
	uint8_t root_depth = root_iter.getDepth();
	if (root_depth < m_octree->getTreeDepth())
	  {
	    float root_offset = getNodeOffset(root_depth);
	    int over_dim[3];
	    over_dim[0] = getIterMax(root_center.x(), root_offset, center.x(), kernel_dim[0]);
	    over_dim[1] = getIterMax(root_center.y(), root_offset, center.y(), kernel_dim[1]);
	    over_dim[2] = getIterMax(root_center.z(), root_offset, center.z(), kernel_dim[2]);
	    
	    point3d center_iter;
	    for (int i = 0; i < over_dim[0]; i++)
	      {
		for (int j = 0; j < over_dim[1]; j++)
		  {
		    for (int k = 0; k < over_dim[2]; k++)
		      {
			center_iter = point3d(i*m_octree->getResolution() + root_center.x(),
					      j*m_octree->getResolution() + root_center.y(),
					      k*m_octree->getResolution() + root_center.z());
			score = calculateTraversability(center_iter) / kernel_size;
      root_iter->setTraversability(score);

			pcl::PointXYZI pnt_iter;
			pnt_iter.x = center_iter(0);
			pnt_iter.y = center_iter(1);
			pnt_iter.z = center_iter(2);

			pnt_iter.intensity = score;

			changedCellsKernel.push_back(pnt_iter);
		      }
		  }
	      }

	    // return;
	  }
	else
	  {
	    score = calculateTraversability(root_center) / kernel_size;
	    root_iter->setTraversability(score);

	    pcl::PointXYZI pnt_iter;
	    pnt_iter.x = root_center(0);
	    pnt_iter.y = root_center(1);
	    pnt_iter.z = root_center(2);

	    pnt_iter.intensity = score;

	    changedCellsKernel.push_back(pnt_iter);
	  }
	      
	
	

      }
  }
  
  void TraverseOctomapServer::insertScan(const tf::Point & sensorOrigin, const PCLPointCloud & ground, const PCLPointCloud & nonground) {
    OctomapServer::insertScan(sensorOrigin, ground, nonground);

    //printValue();

    if (track_changes) {
      trackChanges();
    }
  }

  void TraverseOctomapServer::printValue() {
    KeyBoolMap::const_iterator startPnt = m_octree->changedKeysBegin();
    KeyBoolMap::const_iterator endPnt = m_octree->changedKeysEnd();
    for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) {
      TraversabilityOcTreeNode* node = m_octree->search(iter->first);
      std::cout << node->getValue() << std::endl;
    }
  }

  void TraverseOctomapServer::trackChanges() {
    KeyBoolMap::const_iterator startPnt = m_octree->changedKeysBegin();
    KeyBoolMap::const_iterator endPnt = m_octree->changedKeysEnd();

    //ROS_WARN("Changes Detected: %zu\n", m_octree->numChangesDetected());
  
    pcl::PointCloud<pcl::PointXYZI> changedCells = pcl::PointCloud<pcl::PointXYZI>();
    changedCellsKernel = pcl::PointCloud<pcl::PointXYZI>();

    int c = 0;
    for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) {
      ++c;
      TraversabilityOcTreeNode* node = m_octree->search(iter->first);

      //std::cout << node->getTraversability().proba << std::endl;

      bool occupied = m_octree->isNodeOccupied(node);

      point3d center = m_octree->keyToCoord(iter->first);

      pcl::PointXYZI pnt;
      pnt.x = center(0);
      pnt.y = center(1);
      pnt.z = center(2);

      if (occupied) {
	pnt.intensity = 1000;
      }
      else {
	pnt.intensity = -1000;
      }

      changedCells.push_back(pnt);

      exhaustiveKernel(center);
    }      

    if (c > min_change_pub)
      {
	sensor_msgs::PointCloud2 changed;
	pcl::toROSMsg(changedCells, changed);
	changed.header.frame_id = change_id_frame;
	changed.header.stamp = ros::Time().now();
	pubChangeSet.publish(changed);
	ROS_DEBUG("[server] sending %d changed entries", (int)changedCells.size());

	sensor_msgs::PointCloud2 changedKernel;
	pcl::toROSMsg(changedCellsKernel, changedKernel);
	changedKernel.header.frame_id = change_id_frame;
	changedKernel.header.stamp = ros::Time().now();
	pubChangeSetKernel.publish(changedKernel);
	ROS_DEBUG("[server] sending %d changed entries", (int)changedCellsKernel.size());

	m_octree->resetChangeDetection();
	ROS_DEBUG("[server] octomap size after updating: %d", (int)m_octree->calcNumNodes());
      }
  }

  void TraverseOctomapServer::trackCallback(sensor_msgs::PointCloud2Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZI> cells;
    pcl::fromROSMsg(*cloud, cells);
    ROS_DEBUG("[client] size of newly occupied cloud: %i", (int)cells.points.size());

    for (size_t i = 0; i < cells.points.size(); i++) {
      pcl::PointXYZI& pnt = cells.points[i];
      m_octree->updateNode(m_octree->coordToKey(pnt.x, pnt.y, pnt.z), pnt.intensity, false);
    }

    m_octree->updateInnerOccupancy();
    ROS_DEBUG("[client] octomap size after updating: %d", (int)m_octree->calcNumNodes());
  }

} /* namespace octomap */
