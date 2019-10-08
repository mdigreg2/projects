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

#ifndef OCTOMAP_SERVER_TRAVERSEOCTOMAPSERVER_H_
#define OCTOMAP_SERVER_TRAVERSEOCTOMAPSERVER_H_

#include "TraversabilityOctomapServer.h"

namespace octomap_server {

class TraverseOctomapServer: public OctomapServer {
public:
  TraverseOctomapServer(const std::string& filename = "");
  virtual ~TraverseOctomapServer();

  void trackCallback(sensor_msgs::PointCloud2Ptr cloud);
  void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

protected:
  void trackChanges();
  void printValue();
  // int* getIterStart(const octomap::point3d& center, const octomap::point3d& offset);
  // int* getIterEnd(const octomap::point3d& center, const octomap::point3d& offset);
  int getIterMin(const float& center, const float& offset, const float& root_center);
  int getIterMax(const float& center, const float& offset, const float& root_center, const int& k_max);
  void updateNodeScore(const float& score, const octomap::OcTreeNode& node);
  void exhaustiveKernel(const octomap::point3d& center);
  float calculateTraversability(const octomap::point3d& center);
  inline float getNodeOffset(float depth) {
    return (m_octree->getResolution() / 2) - (m_octree->getNodeSize(depth) / 2);
  }

  bool listen_changes;
  bool track_changes;
  int min_change_pub;
  double robot_length;
  double robot_width;
  double robot_height;
  int kernel_dim[3];
  octomap::point3d kernel_offset;
  float ***kernel;
  int kernel_size;
  pcl::PointCloud<pcl::PointXYZI> changedCellsKernel;
  std::string change_id_frame;
  ros::Publisher pubFreeChangeSet;
  ros::Publisher pubChangeSet;
  ros::Publisher pubChangeSetKernel;
  ros::Subscriber subChangeSet;
  ros::Subscriber subFreeChanges;
};

} /* namespace octomap */
#endif /* TRAVERSEOCTOMAPSERVER_H_ */
