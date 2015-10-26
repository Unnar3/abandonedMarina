//This program filters the map using algorithms in Probablist Robotics chapter 6
#include <ros/ros.h>
  // PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include "SensorData/readISLOG.h"

#include <math.h> 
#include <vector>
#include <iostream>
#include <fstream>

#define outputScan			"Detectedfeatures"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



using namespace std;
using namespace Eigen;

class createBagFile{
public:
    ros::NodeHandle nh;

private:
	ros::Publisher pubScan;
	// readISLOG::isData isDataVec;


public:
	createBagFile(){
		nh = ros::NodeHandle("~");
		pubScan = nh.advertise<PointCloudT>(outputScan, 1);
		// pubScan = nh.advertise<sensor_msgs::PointCloud2>(outputScan, 1);


		// For some stupid reason this is needed so that I don't get linker error for 
		// Extract indices and passThroughFilter and transformPointCloud.
		pcl::ExtractIndices<PointT> extract;
		pcl::PassThrough<PointT> pass;
		PointCloudT::Ptr tmp (new PointCloudT());
		tmp->header.frame_id="odom";
		// pcl_ros::transformPointCloud("/odom" , *tmp, *tmp, listener);
		
		readScan();
	}

	void readScan(){
		ReadISLOG islog;
		ReadISLOG::isData isDataVec = islog.readISLOG();
		PointCloudT::Ptr scan (new PointCloudT());
		ros::Time begin;
		PointT p;
		double firstTime = isDataVec.logtime[0];
		for(int i = 0; i < isDataVec.logtime.size(); ++i){
			// std::cout << "hmmmm" << std::endl;
			scan->clear();
			scan->header.frame_id = "sonar";
			std::cout << isDataVec.logtime[i] << std::endl;
			// scan->header.stamp = isDataVec.logtime[i];
			for(int j = 0; j < isDataVec.bins[i].size(); ++j){
				p.x = 0;
				p.z = 0;
				p.y = j*0.1;
				p.intensity = isDataVec.bins[i][j];
				scan->points.push_back(p);
			}
			pcl_conversions::toPCL(ros::Time(isDataVec.logtime[i]), scan->header.stamp);
			// sensor_msgs::PointCloud2 output;
	  //       pcl::toROSMsg(*scan, output);
	  //       output.header.stamp = ros::Time(isDataVec.logtime[i]);
        	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  			transform.rotate (Eigen::AngleAxisf (isDataVec.transducerangle[i], Eigen::Vector3f::UnitZ()));
  			pcl::transformPointCloud (*scan, *scan, transform);
			pubScan.publish(scan);
		}
		std::cout << "size: " << isDataVec.logtime.size() << std::endl;
	}


};

int main (int argc, char** argv)
{
	// Initialize ROS 
	ros::init (argc, argv, "abandonedmarina");

	createBagFile cbf;
	// Spin
	ros::spin ();
	return(0);
}