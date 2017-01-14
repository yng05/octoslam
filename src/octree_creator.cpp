/*
*
*OCTOSLAM
*Octoslam is a ROS package which converts files in Polygon File Format(ply) to Binary Terrain Files(bt).
*This software has been developed by using Octomap- An Efficient Probabilistic 3D Mapping Framework Based on Octrees as infrastructure.
*http://octomap.github.com/.
*
*To find detailed information please visit following address
*https://github.com/yng05/octoslam/wiki
*
*Bauhaus University Weimar 2016-17
*
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
OcTree tree (0.1);

void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

void grow_map(const PointCloud::ConstPtr& pCloud){

  BOOST_FOREACH (const pcl::PointXYZ& pt, pCloud->points){
    point3d endpoint ((float) pt.x, (float) pt.y, (float) pt.z);
    tree.updateNode(endpoint, true); me
  }

  
  point3d query (0., 0., 0.);
  OcTreeNode* result = tree.search (query);
  print_query_info(query, result);

  query = point3d(-1.,-1.,-1.);
  result = tree.search (query);
  print_query_info(query, result);

  query = point3d(1.,1.,1.);
  result = tree.search (query);
  print_query_info(query, result);
}

void pointCloudCallback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  cout << msg->header.frame_id << endl;
  grow_map(msg);  
}

void doneMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  cout << msg->data.c_str() << endl;

  octomap_msgs::Octomap bmap_msg;
  octomap_msgs::binaryMapToMsg(tree, bmap_msg);
  ros::NodeHandle n;

  ros::Publisher octomap_publisher = n.advertise<octomap_msgs::Octomap>("octree",1);


  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    octomap_publisher.publish(bmap_msg);
    loop_rate.sleep();
  }

}

int main(int argc, char** argv) {
  cout << "listening point cloud publisher..." << endl;
  ros::init(argc, argv, "octree_creator");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("pointcloud", 1000, pointCloudCallback);
  ros::Subscriber subDone = n.subscribe("donemessage", 1000, doneMsgCallback);
  ros::spin();

  return 0;

}

