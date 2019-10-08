/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
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

#include<TraversabilityOcTree.h>
#include<math.h>

namespace octomap {

//================================================================================
  //node implementation
//================================================================================
//--------------------------------------------------------------------------------
  std::ostream& TraversabilityOcTreeNode::writeData(std::ostream &s) const {
    s.write((const char*) &value, sizeof(value)); //occupancy
    s.write((const char*) &proba, sizeof(Traversability)); //traversability

    return s;
  }

  std::istream& TraversabilityOcTreeNode::readData(std::istream &s) {
    s.read((char*) &value, sizeof(value)); //occupancy
    s.read((char*) &proba, sizeof(Traversability)); //traversability
    return s;
  }
  //--------------------------------------------------------------------------------
  TraversabilityOcTreeNode::Traversability TraversabilityOcTreeNode::getAverageChildTraversability() const{
    int mproba = 0;
    int counter = 0;

    if(children != NULL){
      for(int i=0; i<8; i++) {
	TraversabilityOcTreeNode* child = static_cast<TraversabilityOcTreeNode*>(children[i]);
        
	if (child != NULL && child->isTraversabilitySet()) {
	  mproba += child->getTraversability().proba;
	  ++counter;
	}
      }
    }

    if(counter>0) {
      mproba /= counter;
      return Traversability((float) mproba);
    }
    //no child has a traversability thats not 100%
    else {
      return Traversability(2.5);
    }
  }//getAverageChildTraversability()
//--------------------------------------------------------------------------------
  void TraversabilityOcTreeNode::updateTraversabilityChildren() {
    proba = getAverageChildTraversability();
  }
//--------------------------------------------------------------------------------
//================================================================================
  //tree implementation
//================================================================================
//--------------------------------------------------------------------------------
  TraversabilityOcTree::TraversabilityOcTree(double in_resolution, float r_length, float r_width, float r_height) : OccupancyOcTreeBase<TraversabilityOcTreeNode>(in_resolution){
    traversabilityOcTreeMemberInit.ensureLinking();

    robot_length = r_length;
    robot_width = r_width;
    robot_height = r_height;

    // Number of voxels in each dimension of kernel
    kernel_dim[0] = ceilf(r_length / in_resolution);
    kernel_dim[1] = ceilf(r_width / in_resolution);
    kernel_dim[2] = ceilf(r_height / in_resolution) + 1; // +1 here for additional height voxel row

    // Size of kernel in total voxels
    kernel_size = kernel_dim[0] * kernel_dim[1] * kernel_dim[2];
    
    // Kernel offset is kernel dimensions in meters minus root voxel
    kernel_offset = point3d((kernel_dim[0] - 1) * in_resolution,
			    (kernel_dim[1] - 1) * in_resolution,
			    (kernel_dim[2] - 1) * in_resolution);

    score_factor = 1 / ( this->getClampingThresMax() - this->getClampingThresMin() );
    
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
	    for (int k = 0; k < kernel_dim[2]; k++)
	      {
		kernel[i][j][k] = this->getClampingThresMin();
	      }
	    kernel[i][j][0] = this->getClampingThresMax();
	  }
      }

  };
//--------------------------------------------------------------------------------
  TraversabilityOcTreeNode* TraversabilityOcTree::setNodeTraversability(const OcTreeKey& key, float proba){
    //point at the node with the specified key
    TraversabilityOcTreeNode* n = search(key);
    if(n != 0) {
      n->setTraversability(proba);
    }
    return n;
  }
//--------------------------------------------------------------------------------
  bool TraversabilityOcTree::pruneNode(TraversabilityOcTreeNode* node){
    //if its not collapsible
    if(!isNodeCollapsible(node))
      return false;
    
    //set value to childrens values, all assumed equal
    node->copyData(*(getNodeChild(node, 0)));

    //check if trav is set
    if(node->isTraversabilitySet())
      node->setTraversability(node->getAverageChildTraversability());
    
    //delete children
    for(unsigned int i=0; i<8; i++) {
      deleteNodeChild(node, i);
    }
    delete[] node->children;
    node->children = NULL;

    return true;
  }//pruneNode
//--------------------------------------------------------------------------------
  bool TraversabilityOcTree::isNodeCollapsible(const TraversabilityOcTreeNode* node) const{
    //all children must exist
    //must not have children of their own and have the exact same occupancy probability
    if(!nodeChildExists(node, 0))
      return false;
    
    const TraversabilityOcTreeNode* firstChild = getNodeChild(node, 0);
    if(nodeHasChildren(firstChild))
      return false;
    
    for(unsigned int i=1; i<8; i++) {
      //compare nodes using only occupancy for the sake of pruning
      //if it exists, has no children, or has a different value than the first child return false
      if(!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue()) || !(getNodeChild(node, i)->getTraversability() == firstChild->getTraversability()))
	return false;
    }

    return true;
  }
//--------------------------------------------------------------------------------
  TraversabilityOcTreeNode* TraversabilityOcTree::averageNodeTraversability(const OcTreeKey& key, float proba){

    //search for the key and store its address
    TraversabilityOcTreeNode* n = search(key);
    if(n != 0 ){
      if(n->isTraversabilitySet()) {
	TraversabilityOcTreeNode::Traversability prev_trav = n->getTraversability();
	n->setTraversability((prev_trav.proba + proba)/2);
      }
      else{
	n->setTraversability(proba);
      }
    }
    return n;
  }
//--------------------------------------------------------------------------------
  TraversabilityOcTreeNode* TraversabilityOcTree::integrateNodeTraversability(const OcTreeKey& key, float proba){
    TraversabilityOcTreeNode* n = search(key);
    if(n != 0){
      if(n->isTraversabilitySet()){
	TraversabilityOcTreeNode::Traversability prev_trav = n->getTraversability();
	double node_prob = n->getOccupancy();
	float new_proba = (float) ((double) prev_trav.proba * node_prob + (double) proba * (0.99-node_prob));
	n->setTraversability(new_proba);
      }
      else{
	n->setTraversability(proba);
      }
    }
    return n;
  }

  int TraversabilityOcTree::getIterMin(const float& center, const float& offset, const float& root_center) {
    float min_ind = center - offset;
    if (min_ind < 0)
      min_ind = 0;
    min_ind -= root_center;
    return (int)roundf(min_ind);
  }
  
  int TraversabilityOcTree::getIterMax(const float& center, const float& offset, const float& root_center, const int& k_max) {
    float max_ind = center + offset;
    if (max_ind > k_max)
      max_ind = k_max;
    max_ind -= root_center;
    return (int)roundf(max_ind);
  }

  // NODE* TraversabilityOcTree::getLeafNode (const OcTreeKey& key) const {
  //   if (root == NULL)
  //     return NULL;

  //   // generate appropriate key_at_depth for queried depth
  //   OcTreeKey key_at_depth = key;
  //   NODE* curNode (root);
  //   int diff = 0;

  //   // follow nodes down to requested level (for diff = 0 it's the last level)
  //   for (int i=(tree_depth-1); i>=diff; --i) {
  //     unsigned int pos = computeChildIdx(key_at_depth, i);
  //     if (nodeChildExists(curNode, pos)) {
  //       // cast needed: (nodes need to ensure it's the right pointer)
  //       curNode = getNodeChild(curNode, pos);
  //     } else {
  //       // we expected a child but did not get it
  //       // is the current node a leaf already?
  //       if (!nodeHasChildren(curNode)) { // TODO similar check to nodeChildExists?
  // 	  m_octree->expandNode(curNode);
  // 	  curNode = getNodeChild(curNode, pos);
  //       } else {
  //         // it is not, search failed
  //         return NULL;
  //       }
  //     }
  //   } // end for
  //   return curNode;
  // }  
  
  void TraversabilityOcTree::exhaustiveKernel(const point3d& center) {
    for (TraversabilityOcTree::true_leaf_bbx_iterator root_iter = true_leaf_bbx_iterator(this,
											 center - kernel_offset,
											 center);
	 root_iter.isStackEmpty() != true;
	 ++root_iter)
      {
	// Initialize score values
	int voxels_hit = 0;
	float score = 0.0;

	point3d root_center = root_iter.getCoordinate();
	score = 1 - ( calculateTraversability(root_center) / kernel_size );
	root_iter->setTraversability(score);
      }
  }


  float TraversabilityOcTree::calculateTraversability(const point3d& center) {

    float score = 0;
    int voxels_hit = 0;
	
    for (TraversabilityOcTree::leaf_bbx_iterator kernel_it = this->begin_leafs_bbx(center,
								      center + kernel_offset,
								      this->getTreeDepth()),
	   kernel_end = this->end_leafs_bbx(); kernel_it != kernel_end; ++kernel_it)
      {
	point3d kernel_center = kernel_it.getCoordinate();
	point3d ind = kernel_center - center;
	float value = kernel_it->getOccupancy();
	uint8_t depth = kernel_it.getDepth();

	if (depth < this->getTreeDepth())
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
			score += fabs(kernel[i][j][k] - value);
			voxels_hit++;
		      }
		  }
	      }
	  }
	else
	  {
	    // Perform kernel comparison
	    score += fabs(kernel
			      [ (int)roundf(ind.x()/this->getResolution()) ]
			      [ (int)roundf(ind.y()/this->getResolution()) ]
			      [ (int)roundf(ind.z()/this->getResolution()) ] - value) * score_factor;
	    // Increment count of voxels hit
	    voxels_hit++;
	  }
      }
    // Add uncertain values that were never iterated
    score += (kernel_size - voxels_hit) * 0.5;
    return score;
  }

//--------------------------------------------------------------------------------
  void TraversabilityOcTree::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }
//--------------------------------------------------------------------------------
  void TraversabilityOcTree::updateInnerOccupancyRecurs(TraversabilityOcTreeNode* node, unsigned int depth){
    //only recurse and update for the inner nodes
    if(nodeHasChildren(node)){
      //return early for last level
      if(depth < this->tree_depth){
	for(unsigned int i=0; i<8; i++){
	  if(nodeChildExists(node, i)){
	    updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
	  }
	}
      }
      node->updateOccupancyChildren();
      node->updateTraversabilityChildren();
    }
  }
//--------------------------------------------------------------------------------
  std::ostream& operator<<(std::ostream& out, TraversabilityOcTreeNode::Traversability const& c){
    return out << '(' << (unsigned int)c.proba << ')';
  }
//--------------------------------------------------------------------------------
  int TraversabilityOcTree::getKernelSize(){
  	return(this->kernel_size);
  }
//--------------------------------------------------------------------------------
  point3d TraversabilityOcTree::getKernelOffset(){
  	return(this->kernel_offset);
  }
//--------------------------------------------------------------------------------
  int* TraversabilityOcTree::getKernelDim(){
  	return(this->kernel_dim);
  }
//--------------------------------------------------------------------------------
  TraversabilityOcTree::StaticMemberInitializer TraversabilityOcTree::traversabilityOcTreeMemberInit;
//--------------------------------------------------------------------------------
} //namespace octomap

