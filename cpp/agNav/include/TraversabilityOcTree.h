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


//functions that must be defined in the .cpp file
//under aggressive-navgation/src/traversabilityoctomap
/*
getAverageChildTraversability()
updateTraversabilityChldren()
readData(std::istream &s);
writeData(std::ostream &s) const;
setNodeTraversability(OcTreeKey& key, float proba) 
averageNodeTraversability(OcTreeKey& key, float proba)
updateInnterOccupancy()
protected: void updateInnerOccupancyRecurs(TraversabilityOcTreeNode* node, unsigned int depth);
 static StaticMemberInitializer tarversabilityOcTreeMemberInit;
 
 CHECK LINE 165 FOR COVAFIANT RETURN TYPE OF TREE CONSTRUCTOR
 */



#ifndef OCTOMAP_TRAVERSABILITY_OCTREE_H
#define OCTOMAP_TRAVERSABILITY_OCTREE_H

#include<iostream>
#include<octomap/OcTreeNode.h>
#include<octomap/OccupancyOcTreeBase.h>
#include<ros/ros.h>

namespace octomap {

    //forward declaration for friend
    class TraversabilityOcTree;

    //define the node. inherits from OcTreeNode
    class TraversabilityOcTreeNode : public OcTreeNode {
        public:
        //TravOcTree needs friend access to its children
            friend class TraversabilityOcTree;    

            //define a class where we define what traversability is   in this case a probability called proba
            class Traversability {
                public:

                
                Traversability() : proba(0) {}
                Traversability(float _proba) : proba(_proba) {}

                //define the operators ==, !=, >=, <=, >, < to mean comparing the proba values of nodes
                inline bool operator== (const Traversability &other) const {
                    return (proba==other.proba);
                }
                inline bool operator!= (const Traversability &other) const {
                    return (proba!=other.proba);
                }
                inline bool operator>= (const Traversability &other) const {
                    return (proba>=other.proba);
                }
                inline bool operator<= (const Traversability &other) const {
                    return (proba<=other.proba);
                }
                inline bool operator> (const Traversability &other) const {
                    return (proba>other.proba);
                }
                inline bool operator< (const Traversability &other) const {
                    return (proba<other.proba);
                }
                float proba;
            }; //class Traversability

        //still the public domain of TraversabilityOcTreeNode but we will find out when I compile
        //weird errors mean add public: here
        public:
        TraversabilityOcTreeNode() : OcTreeNode() {}

        //when you make a traversability oc tree node you are necesarrily making an oc tree node with an extension of proba
        TraversabilityOcTreeNode(const TraversabilityOcTreeNode& ton) : OcTreeNode(ton), proba(ton.proba) {}

        //when you compare to see if two nodes are equal with the == operator check the OcTreeNode value and the probas, 
        //two nodes are only equal with == if their probas and their values are equal. This goes the same for the other operators
        //defined after the == operator is
        bool operator== (const TraversabilityOcTreeNode& ton) const {
            return(ton.value == value && ton.proba == proba);
        }
        bool operator!= (const TraversabilityOcTreeNode& ton) const {
            return(ton.value != value && ton.proba != proba);
        }
        bool operator>= (const TraversabilityOcTreeNode& ton) const {
            return(ton.value >= value && ton.proba >= proba);
        }
        bool operator<= (const TraversabilityOcTreeNode& ton) const {
            return(ton.value <= value && ton.proba >= proba);
        }
        bool operator> (const TraversabilityOcTreeNode& ton) const {
            return(ton.value > value && ton.proba < proba);
        }
        bool operator< (const TraversabilityOcTreeNode& ton) const {
            return(ton.value < value && ton.proba < proba);
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void copyData(const TraversabilityOcTreeNode& ton) {
            OcTreeNode::copyData(ton);
            this->proba = ton.getTraversability();
        }

        inline Traversability getTraversability() const {return proba;}
        inline void setTraversability(Traversability t) {this->proba = t;}
        inline void setTraversability(float proba) {this->proba = Traversability(proba);}

        Traversability& getTraversability() {return proba;}

        inline bool isTraversabilitySet() const {
            return (proba != 0);   
        }

        void updateTraversabilityChildren();

        TraversabilityOcTreeNode::Traversability getAverageChildTraversability() const;

        //file I/O enable
        std::istream& readData(std::istream &s);
        std::ostream& writeData(std::ostream &s) const;

    protected:
        Traversability proba;
    }; //class TraversabilityOcTreeNode


        //Define the new tree that will use this node
        class TraversabilityOcTree : public OccupancyOcTreeBase <TraversabilityOcTreeNode> {
	public:

          #include<OcTreeTrueLeafIterator.hxx>

	  // same as the default constructor, it sets leaf resolution
          // also includes dimensions for raw kernel body, pulled from
          // robot_{dimension} params

          TraversabilityOcTree(double in_resolution, float r_length=0.5, float r_width=0.5, float r_height=0.5);

          // virtual constructor: creates a new object of the same type
          // Convariant return type requires an up-to-date compiler, it's 2019
          // at the time of writing
          TraversabilityOcTree *create() const {
            return new TraversabilityOcTree(resolution, robot_length, robot_width, robot_height); }

            //support for getTreeType()
            std::string getTreeType() const { return "TraversabilityOcTree"; }

            //for puruning purposes
            /*
            Overloaded. Prunes node when it is collapsable 
            only considers node occupancy for pruning
            different traversabilities ignored. This functionality may have 
            to be extended later. return true if the pruning was successful
             */
            virtual bool pruneNode(TraversabilityOcTreeNode* node);

            virtual bool isNodeCollapsible(const TraversabilityOcTreeNode* node) const;

            //set traversability at a given coordinate, replaces previous traversability
            //takes an OcTreeKey to delineate the node
            TraversabilityOcTreeNode* setNodeTraversability(const OcTreeKey& key, float proba);

            //takes XYZ coordinates as a float
            //checks to see if it can convert the coordinates to an OcTreeKey
            //if it can it calls the previous function and sets trav. otherwise it returns null
            TraversabilityOcTreeNode* setNodeTraversability(float x, float y, float z, float proba){
                OcTreeKey key;
                if(!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
                return setNodeTraversability(key, proba);
            }

            //same functional structure as above 
            TraversabilityOcTreeNode* averageNodeTraversability(const OcTreeKey& key, float proba);
            TraversabilityOcTreeNode* averageNodeTraversability(float x, float y, float z, float proba){
                OcTreeKey key;
                if(!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
                return averageNodeTraversability(key, proba);
            }

            //same functional structure as above
            TraversabilityOcTreeNode* integrateNodeTraversability(const OcTreeKey& key, float proba);
            TraversabilityOcTreeNode* integrateNodeTraversability(float x, float y, float z, float proba){
                OcTreeKey key;
                if(!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
                return integrateNodeTraversability(key, proba);
            }

            //updates inner nodes, sets the trav to average trav of children
            void updateInnerOccupancy();

	  // Traversability calculation functions
	  void exhaustiveKernel(const point3d& center);
	  float calculateTraversability(const point3d& center);
	  int getIterMin(const float& center, const float& offset, const float& root_center);
	  int getIterMax(const float& center, const float& offset, const float& root_center, const int& k_max);
	  inline float getNodeOffset(float depth) {
	    return (this->getResolution() / 2) - (this->getNodeSize(depth) / 2);
	  }

      int getKernelSize();
      octomap::point3d getKernelOffset();
      int* getKernelDim();

        protected:
            void updateInnerOccupancyRecurs(TraversabilityOcTreeNode* node, unsigned int depth);

            /**
             * Static Member object which ensure that the OcTree's prototype is 
             * represented in classIDMapping only once. needed as a static member in 
             * any derived OcTree class in order to read .ot files 
             * through the AbstractOcTree factory. should also call ensureLinking()
             * once from the constructor
             */

            class StaticMemberInitializer{
                public:
                StaticMemberInitializer() {

		  TraversabilityOcTree* tree = new TraversabilityOcTree(0.1);
                    tree->clearKeyRays();
                    AbstractOcTree::registerTreeType(tree);
                }

                /**
                 * Dummy function to ensure that the MSVC does not 
                 * drop the StaticMemberInitializer causing this tree failing to register
                 * Needs to be called from the constructor of this octree
                 */
                void ensureLinking() {};

            }; //class StaticMemberInitializer
            //static member to ensure static initialization (only once)
        static StaticMemberInitializer traversabilityOcTreeMemberInit;

	private:
	  int kernel_dim[3];
	  octomap::point3d kernel_offset;
	  float ***kernel;
	  int kernel_size;
	  float robot_length;
	  float robot_width;
	  float robot_height;
	  float score_factor;

        }; //class TraversabilityOcTree

        //output format (proba)
        std::ostream& operator<<(std::ostream& out, TraversabilityOcTreeNode::Traversability const &t);
} //namespace octomap


#endif
