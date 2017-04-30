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

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <std_msgs/String.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudXRN;
typedef sensor_msgs::PointCloud2 PointCloud2;
ros::Subscriber sub;
int countRecieved = 0;

void pcReceiverCallback(const PointCloud2::ConstPtr& message)
{
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("pointcloud", 100); 

  PointCloud cloud;
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*message, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, cloud);

  PointCloud pc = cloud;

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "frame_" + boost::lexical_cast<std::string>(countRecieved++);

  msg->height = pc.height;
  msg->width = pc.width;

  for (int j=0; j < pc.size(); j++){
  msg->points.push_back (pcl::PointXYZ(pc.points[j].x,pc.points[j].y,pc.points[j].z));
  }

  ros::Rate loop_rate(5);
  loop_rate.sleep();
  pub.publish(msg);
  ros::spinOnce();
  std::cout << msg->header.frame_id << " package is sent" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcls");
  cout << "listening point cloud publisher..." << endl;
  ros::NodeHandle n;
  sub = n.subscribe("orb_slam/point_cloud", 100, pcReceiverCallback);
  ros::spin();
}
