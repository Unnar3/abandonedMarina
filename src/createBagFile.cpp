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
#include <abandonedmarina/amDVL.h>
#include <abandonedmarina/amMTi.h>

#include <math.h> 
#include <vector>
#include <iostream>
#include <fstream>

#define outputScan			"SonarScan"
#define outputDVL			"DVLData"
#define outputMTi			"MTiData"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



using namespace std;
using namespace Eigen;

class abandonedMarina{
public:
    ros::NodeHandle nh;

private:
	ros::Publisher pubScan;
	ros::Publisher dvlData;
	ros::Publisher mtiData;
	// readISLOG::isData isDataVec;


public:
	abandonedMarina(){
		nh = ros::NodeHandle("~");
		pubScan = nh.advertise<PointCloudT>(outputScan, 0);
		dvlData = nh.advertise<abandonedmarina::amDVL>(outputDVL, 0);
		mtiData = nh.advertise<abandonedmarina::amMTi>(outputMTi, 0);
		// pubScan = nh.advertise<sensor_msgs::PointCloud2>(outputScan, 1);


		// For some stupid reason this is needed so that I don't get linker error for 
		// Extract indices and passThroughFilter and transformPointCloud.
		pcl::ExtractIndices<PointT> extract;
		pcl::PassThrough<PointT> pass;
		PointCloudT::Ptr tmp (new PointCloudT());
		tmp->header.frame_id="odom";
		// pcl_ros::transformPointCloud("/odom" , *tmp, *tmp, listener);
		
		readScan();
		readDVL();
		readMTi();
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
			// std::cout << isDataVec.logtime[i] << std::endl;
			// scan->header.stamp = isDataVec.logtime[i];
			for(int j = 0; j < isDataVec.bins[i].size(); ++j){
				p.x = 0;
				p.z = 0;
				p.y = j*0.1;
				p.intensity = isDataVec.bins[i][j];
				scan->points.push_back(p);
			}
			pcl_conversions::toPCL(ros::Time(isDataVec.logtime[i]), scan->header.stamp);
        	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  			transform.rotate (Eigen::AngleAxisf (isDataVec.transducerangle[i], Eigen::Vector3f::UnitZ()));
  			pcl::transformPointCloud (*scan, *scan, transform);
			pubScan.publish(scan);
		}
		std::cout << "size: " << isDataVec.logtime.size() << std::endl;
	}

	void readDVL(){
		abandonedmarina::amDVL tmp_dvld;

		std::cout << "Started reading DVL data!" << std::endl;

        std::ifstream file("/home/unnar/catkin_ws/src/abandonedmarina/_040825_1735_DVL.log");
        std::string line; 

        int line_number = 0;
        std::cout.precision(13);
        double number = 0;
        // dvld tmp_dvld;
        while (std::getline(file, line))
        {
            // std::cout << "hnenene" << std::endl;
            std::stringstream   linestream(line);
            std::string         data;

            if (line_number > 5){
                if (!line.empty() || line!=""){
                    // std::getline(linestream, data, '\t');  // read up-to the first tab (discard tab).
                    // std::cout << "hmmmmmm" << std::endl;
                    for(int i = 0; i < 32; i++){
                        linestream >> number;
                        if (i == 0){
                            tmp_dvld.header.stamp = ros::Time(number);
                        } else if (i == 7){
                            tmp_dvld.vwx = number;
                        } else if (i == 8){
                            tmp_dvld.vwy = number;
                        } else if (i == 9){
                            tmp_dvld.vwz = number;
                        } else if (i == 10){
                            tmp_dvld.waterok = int(number);
                        } else if (i == 11){
                            tmp_dvld.vbx = number;
                        } else if (i == 12){
                            tmp_dvld.vby = number;
                        } else if (i == 13){
                            tmp_dvld.vbz = number;
                        } else if (i == 14){
                            tmp_dvld.bottomok = int(number);
                        } else if (i == 22){
                        	tmp_dvld.yaw = number/360*2*M_PI;
                        } else if (i == 30){
                        	tmp_dvld.xt = number/360*2*M_PI;
                        } else if (i == 31){
                        	tmp_dvld.yt = number/360*2*M_PI;
                        }
                    }
                    dvlData.publish(tmp_dvld);
                }
            }
            line_number++;
        }
        std::cout << "Finished reading DVL data!" << std::endl;
        std::cout << "lines: " << line_number << std::endl;
	}

	void readMTi(){

		abandonedmarina::amMTi tmp_mti;

		std::cout << "Started reading MTi data!" << std::endl;

        std::ifstream file("/home/unnar/catkin_ws/src/abandonedmarina/_040825_1735_MTi.log");
        std::string line; 


		int line_number = 0;
        std::cout.precision(13);
        double number = 0;

        while (std::getline(file, line))
        {
            std::stringstream   linestream(line);
            std::string         data;

            if (line_number > 6){
                if (!line.empty() || line!=""){
                    // std::getline(linestream, data, '\t');  // read up-to the first tab (discard tab).
                    for(int i = 0; i < 10; i++){
                        linestream >> number;
                        if (i == 0){
                        	tmp_mti.header.stamp = ros::Time(number);
                        } else if (i == 1) {
                        	tmp_mti.Roll = number/180*M_PI;
                        } else if (i == 2) {
                        	tmp_mti.Pitch = number/180*M_PI;
                        } else if (i == 3){
         					tmp_mti.Yaw = number/180*M_PI;                        	
                        } else if (i == 4){
                        	tmp_mti.rX = number;
                        } else if (i == 5){
                        	tmp_mti.rY = number;
                        } else if (i == 6){
                        	tmp_mti.rZ = number;
                        } else if (i == 7){
                        	tmp_mti.aX = number;
                        } else if (i == 8){
                        	tmp_mti.aY = number;
                        } else if (i == 9){
                        	tmp_mti.aZ = number;
                        }
                    }
                    mtiData.publish(tmp_mti);
                    // vmtid.push_back(tmp_mtid);
                }
            }
            line_number++;
        }
        std::cout << "Finished reading MTi data!" << std::endl;
        std::cout << "lines: " << line_number << std::endl;
	}


};

int main (int argc, char** argv)
{
	// Initialize ROS 
	ros::init (argc, argv, "abandonedMarina");

	abandonedMarina ab;
	// Spin
	ros::spin ();
	return(0);
}