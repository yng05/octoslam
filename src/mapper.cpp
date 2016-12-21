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

#include <string> 
#include <sstream> 
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_ros/conversions.h>

using namespace std;
using namespace octomap;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Subscriber sub;

void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

void print_map(OcTree* tree){
  cout << endl;
  cout << "generating result map" << endl;  
  
  tree->writeBinary("result_tree.bt");
  cout << "wrote example file result_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis result_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  
}

void octreeCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg)
{

  cout << "receiving..." << endl;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*octomap_msg);
  octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

  print_map(octree);
  cout << "printed..." << endl;
  sub.shutdown();
  //print_map(msg);  
}

int main(int argc, char** argv) {
  cout << "listening octree publisher..." << endl;
  ros::init(argc, argv, "mapper");
  ros::NodeHandle n;
  sub = n.subscribe("octree", 1000, octreeCallback);
  ros::spin();

  return 0;

}

