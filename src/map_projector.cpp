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
#include <unistd.h>
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

/*void bt_file_publisher(){

ros::AsyncSpinner spinner(1);
ros::NodeHandle nh;

std::cout<<"Reading File"<<std::endl;

octomap::OcTree* octree = new octomap::OcTree("result_tree.bt");
std::cout << "File Read Sccessfully" << std::endl;

octomap_msgs::Octomap bmap_msg;
octomap_msgs::binaryMapToMsg(*octree, bmap_msg);
ros::Publisher octomap_publisher = nh.advertise<octomap_msgs::Octomap>("display_env",1);

octomap_publisher.publish(bmap_msg);
cout << "published from file.bt..." << endl;

}*/

void octomap_publisher(OcTree* octree) {
	cout << "========================" << endl;
	ros::AsyncSpinner spinner(1);
	ros::NodeHandle nh;
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	
	std::cout << " Leaf Nodes: " << octree->getNumLeafNodes() << "\n";
	std::cout << " resolution: " << octree->getResolution() << "\n";


//========================== Visualization of Octree  ===============================================
	
	int idx = 0;

//========================== START Visualization with Markers =======================================

	visualization_msgs::Marker marker[20000];
	uint32_t shape = visualization_msgs::Marker::CUBE;

	for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
		end = octree->end_leafs(); it != end; ++it)
	{   
		
       	
		marker[idx].header.frame_id = "/my_octree";
		marker[idx].header.stamp = ros::Time::now();

		marker[idx].ns = "basic_octree";
    	marker[idx].id = idx;

		marker[idx].type = shape;
		marker[idx].action = visualization_msgs::Marker::ADD;


		std::cout << "Node center: " << it.getCoordinate();
	 	std::cout << " value: " << it->getValue() << "\n";
		marker[idx].pose.position.x = it.getX();
	    marker[idx].pose.position.y = it.getY();
	    marker[idx].pose.position.z = it.getZ();
	    marker[idx].pose.orientation.x = 0.0;
	    marker[idx].pose.orientation.y = 0.0;
	    marker[idx].pose.orientation.z = 0.0;
	    marker[idx].pose.orientation.w = 1.0;
	// %EndTag(POSE)%

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	// %Tag(SCALE)%
	    marker[idx].scale.x = 0.1;
	    marker[idx].scale.y = 0.1;
	    marker[idx].scale.z = 0.1;
	// %EndTag(SCALE)%

	    // Set the color -- be sure to set alpha to something non-zero!
	// %Tag(COLOR)%
	    marker[idx].color.r = 0.0f;
	    marker[idx].color.g = 0.0f;
	    marker[idx].color.b = 1.0f;
	    marker[idx].color.a = 1.0;

	    marker_pub.publish(marker[idx]);

		usleep(9000);
		idx = idx + 1;
		//std::cout << "Node center: " << it.getCoordinate();
		//std::cout << " value: " << it->getValue() << "\n";
	}
	cout << "Now visualize in rviz with marker and set Frame id = my_octree..." << endl;
	marker_pub.shutdown();

    // ========================= END Visualization with Markers ==========================================



	//========================== START Visualization with MarkerArray =====================================
	

	 // Marker array to visualize the octree. It displays the occuplied cells of the octree

	 /*double lowestRes = octree->getResolution();
	 visualization_msgs::MarkerArray octree_marker_array_msg_;

	 ros::Publisher octree_marker_array_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);

	 for (int temp = 0; temp < 40; temp++)
	 {

	 	octree_marker_array_msg_.markers.resize(5000);

	 	for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
	 		end = octree->end_leafs(); idx <= 4999; ++it)
	 	{ 
	 		geometry_msgs::Point cube_center;
	 		 cube_center.x = it.getX();
	         cube_center.y = it.getY();
	         cube_center.z = it.getZ();
	         octree_marker_array_msg_.markers[idx].points.push_back(cube_center);

	         idx = idx + 1;
	 	}


	 	for (unsigned i = 0; i < octree_marker_array_msg_.markers.size(); ++i)
	     {
	       octree_marker_array_msg_.markers[i].header.frame_id = "my_octree";
	       octree_marker_array_msg_.markers[i].header.stamp = ros::Time::now();

	       double size = lowestRes * pow(2,i);

	       std::stringstream ss;
	       ss <<"Level "<<i;
	       octree_marker_array_msg_.markers[i].ns = ss.str();
	       octree_marker_array_msg_.markers[i].id = i;
	       //octree_marker_array_msg_.markers[i].lifetime = ros::Duration::Duration();
	       octree_marker_array_msg_.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
	       octree_marker_array_msg_.markers[i].scale.x = 0.01;
	       octree_marker_array_msg_.markers[i].scale.y = 0.01;
	       octree_marker_array_msg_.markers[i].scale.z = 0.01;
	    
	       octree_marker_array_msg_.markers[i].color.r = 0.0f;
	       octree_marker_array_msg_.markers[i].color.g = 0.0f;
	       octree_marker_array_msg_.markers[i].color.b = 1.0f;
	       octree_marker_array_msg_.markers[i].color.a = 1.0;

	       if (octree_marker_array_msg_.markers[i].points.size() > 0)
	         octree_marker_array_msg_.markers[i].action = visualization_msgs::Marker::ADD;
	       else
	         octree_marker_array_msg_.markers[i].action = visualization_msgs::Marker::DELETE;

	     }

	     octree_marker_array_publisher_.publish(octree_marker_array_msg_);

	    
	     usleep(100000);
	     std::cout << " loop count: " << temp << "\n";
	 }
     cout << "Now visualize in rviz..." << endl;*/
 //    octree_marker_array_publisher_.shutdown();



//======================================End of OCTREE Visualization with MarketArray==================================================

}


void print_map(OcTree* tree) {
	cout << endl;
	cout << "generating result map" << endl;

	tree->writeBinary("result_tree.bt");
	cout << "wrote example file result_tree.bt" << endl << endl;
	cout << "now you can use octovis to visualize: octovis result_tree.bt" << endl;
	cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl << endl;
}

void octreeCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg)
{
	cout << "receiving..." << endl;
	octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*octomap_msg);
	octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

	print_map(octree);
	cout << "printed..." << endl;
	//bt_file_publisher();
	octomap_publisher(octree);
	sub.shutdown();
}

int main(int argc, char** argv) {
	cout << "listening octree publisher..." << endl;
	ros::init(argc, argv, "map_projector");
	ros::NodeHandle n;
	sub = n.subscribe("octreeFrg", 1000, octreeCallback);
	ros::spin();

	return 0;

}

