#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <boost/lexical_cast.hpp>
#include <std_msgs/String.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudXRN;

int main(int argc, char** argv)
{
  std::vector<PointCloud> pCloudVector;
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("pointcloud", 1); 
  ros::Publisher done_msg_pub = nh.advertise<std_msgs::String>("donemessage", 1000); 

  PointCloudXRN pointCloud;
  char *path = NULL;
  path = getcwd(NULL, 0);
  std::string fullpath = path + std::string("/src/octoslam/resources/dbl_orb_pointcloud.ply");
  pcl::io::loadPLYFile(fullpath, pointCloud);



  int packSize = pointCloud.size() / 100;
  int count = 0;

  do{
      int currentCount = count;
      count += packSize;  
      if(count >= pointCloud.size()){
        count = pointCloud.size();
      }
      std::cout << currentCount << " to " << count << " is processing..." << std::endl;

      PointCloud pc;
      for(int i=currentCount;i < count; i++){        
        pc.push_back(pcl::PointXYZ(pointCloud.points[i].x,pointCloud.points[i].y,pointCloud.points[i].z));
      }

      pCloudVector.push_back(pc);

  }while(count != pointCloud.size());

  
  for(int i=0; i < pCloudVector.size(); i++){
    PointCloud pc = pCloudVector[i];

    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "frame_" + boost::lexical_cast<std::string>(i);

    msg->height = pc.height;
    msg->width = pc.width;

    for (int j=0; j < pointCloud.size(); j++){
    msg->points.push_back (pcl::PointXYZ(pc.points[j].x,pc.points[j].y,pc.points[j].z));
    }

    ros::Rate loop_rate(20);
    loop_rate.sleep();
    pub.publish(msg);
    ros::spinOnce();
    std::cout << msg->header.frame_id << " package is sent" << std::endl;
  }

  ros::Rate loop_rate(10);

  std_msgs::String msgDone;
   
  std::stringstream ss;
  ss << "all packs are done" << count;
  msgDone.data = ss.str();


  done_msg_pub.publish(msgDone);  
  ros::spinOnce();  
  loop_rate.sleep();
}