//ros
#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <cmath>

#include <cstdlib>
#include <ctime>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


#include <iostream>
#include <string>
#include <chrono>
#include <fstream>
#include<sstream>
#include <map>
 
#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <boost/thread/thread.hpp>
#include <time.h>

#include <pclomp/ndt_omp.h>

#define random(a,b) (rand()%(b-a+1)+a)  //生成随机数

using namespace std;

int FRAMENUMBER, INTERVAL;
Eigen::Matrix4f Ti, TempT;

ros::Publisher pcPublisher;   

int global_count, frame_count;

std::vector<int> colorlist;
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_temp_cloud;

std::string pkg_loc = ros::package::getPath("pcl_reg");


//过滤
void cloudFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, bool downsample = true)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud (new pcl::PointCloud<pcl::PointXYZ>), index_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	//removing outlier points
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	pcl::PointIndices inlier;
	for (size_t i=0; i < cloud->points.size(); ++i)
	{
		if   (cloud->points[i].x > -30    && cloud->points[i].x < 30
		  &&  cloud->points[i].y > -30    && cloud->points[i].y < 30
		  &&  cloud->points[i].z > -5    && cloud->points[i].z < 5 )
		// if   ( sqrt(pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2)) < 20 
		// &&  cloud->points[i].z > -2   && cloud->points[i].z < 1.5 )
		{
			inlier.indices.push_back(i);
		}
	}
	extract.setIndices(boost::make_shared<pcl::PointIndices>(inlier));
	extract.setNegative(false);
	extract.filter(*index_cloud);
	
	if (downsample)
	{
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(index_cloud);
		sor.setLeafSize(0.2f, 0.2f, 0.2f);
		sor.filter(*cloud_filtered);
//		std::cerr << "PointCloud after voxel filtering: " << sor_cloud->points.size() << " data points" << std::endl;
	}
	else
	{
		cloud_filtered = index_cloud;
	}

	//find plane coefficients
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	// you can modify the parameter below
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.3);
	// extract ground
	pcl::ExtractIndices<pcl::PointXYZ> extractor;

	
	for (int i =0; i< 2 ; ++i)
	{
		//inliers & coefficients
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			cout<<"error! Could not found any inliers!"<<endl;
			break;
		}
		extractor.setInputCloud(cloud_filtered);
		extractor.setIndices(inliers);
		extractor.setNegative(true);
		extractor.filter(*sor_cloud);
		cloud_filtered.swap(sor_cloud);
	}
	// std::cout << "filter done."<< std::endl;
	// std::cerr << "PointCloud after filtering: " << cloud_filtered->points.size() << " data points" << std::endl;
}


void coloredPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc1, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc2)
{
	int r,g,b;
	r = random(0, 1)*255;  
	g = random(0, 1)*255;
	b = random(0, 1)*255;

	// addRGB(pc1, *pc2, r,g,b);
	for (size_t i=0;i<pc1->points.size();i++)
	{

		pcl::PointXYZRGB p;
		p.x = pc1->at(i).x;
		p.y = pc1->at(i).y;
		p.z = pc1->at(i).z;
		p.r = 255;
		p.g = 0;
		p.b = 0;
		pc2->push_back(p);
	}
	pc2->width = 1;
	pc2->height = pc1->points.size();

}


//----------------加速的ndt
void pclomp_ndtRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr &src, pcl::PointCloud<pcl::PointXYZ>::Ptr &dst)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr out (new pcl::PointCloud<pcl::PointXYZ>);

	pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
	ndt_omp->setResolution(1);
	ndt_omp->setTransformationEpsilon (1e-6);   //为终止条件设置最小转换差异
	ndt_omp->setStepSize (0.2);    //为more-thuente线搜索设置最大步长
	ndt_omp->setMaximumIterations (200);
	ndt_omp->setNumThreads(32);
	ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);  //pclomp::KDTREE   pclomp::DIRECT1

	ndt_omp->setInputSource (src);  //源点云
	// Setting point cloud to be aligned to.
	ndt_omp->setInputTarget (dst);  //目标点云
	ndt_omp->align(*out);// not use init_guess

	// std::cout << "has converged:  " << ndt_omp->hasConverged() << "  score: " << ndt_omp->getFitnessScore() << std::endl;
	// std::cout << ndt_omp->getFinalTransformation() << std::endl;
    
	Ti = Ti * ndt_omp->getFinalTransformation();
	TempT = ndt_omp->getFinalTransformation();
}

void Callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)   //不订阅camerainfo的回调函数
{

//全局计数
	global_count += 1;	
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	sensor_msgs::PointCloud2 msg_clouds;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*pc_msg, *pointcloud);

	cloudFilter(pointcloud, pointcloud, false);  //过滤

	if ( temp_cloud->points.size() == 0 )  
	{
		temp_cloud = pointcloud;
		coloredPoints(temp_cloud, colored_temp_cloud);	
	}

	if (global_count % INTERVAL == 0)
	{
		frame_count +=1;

		ROS_INFO_STREAM(" The " << global_count << " frame\n");
		ROS_INFO_STREAM("PointCloud scan received at " << pc_msg->header.stamp << "ns");

		pclomp_ndtRegistration(pointcloud, temp_cloud);  //点云配准， 外参矩阵

		*temp_cloud = *pointcloud;

		pcl::transformPointCloud(*pointcloud, *output_cloud, Ti);  

		coloredPoints(output_cloud, colored_output_cloud);

		*colored_temp_cloud += *colored_output_cloud;

		if ( frame_count == FRAMENUMBER)
		{
			std::stringstream ss;
			ss << pkg_loc+"final.pcd";
			std::cout << ss.str();
			pcl::io::savePCDFile(ss.str(), *colored_temp_cloud, true);
			ros::shutdown();	
		}

	}
	pcl::toROSMsg(*colored_temp_cloud, msg_clouds);
	msg_clouds.header.frame_id = "pandar";
	msg_clouds.header.stamp = ros::Time::now(); // ADD
	pcPublisher.publish(msg_clouds);

}

int
main (int argc, char** argv)
{
	srand(time(0));  // 产生随机种子 

	// Initialize ROS
	ros::init (argc, argv, "pcl_reg_rgb");
	ros::NodeHandle nh("~");

    std::string lidar_topic, fusion_pc_topic;
	Ti.setIdentity(4,4);
	TempT.setIdentity(4,4);
	

	if (nh.param<std::string>("lidar_topic", lidar_topic, "/pandar_points"))  //订阅点云话题
	// if (nh.getParam("lidar_pc_topic", lidar_pc_topic))
	    ROS_INFO_STREAM("lidar_topic: " << lidar_topic);
    else
		ROS_WARN("Not found lidar_topic"); 

	// nh.param<bool>("bias_flag", bias_flag, false);    //软件偏置标志位

	// if (nh.getParam("camera_pc_topic", camera_pc_topic))   
	if (nh.param<std::string>("fusion_pc_topic", fusion_pc_topic, "/fusion_points"))   //发布 转化到相机坐标系下的点云
	    ROS_INFO_STREAM("fusion_pc_topic: " << fusion_pc_topic);
    else
		ROS_WARN("Not found fusion_pc_topic"); 

	if (nh.param<int>("framenumber", FRAMENUMBER, 10))   //发布 转化到相机坐标系下的点云
	    ROS_INFO_STREAM("framenumber: " << FRAMENUMBER);
    else
		ROS_WARN("Not found framenumber"); 

	if (nh.param<int>("interval", INTERVAL, 5))   //发布 转化到相机坐标系下的点云
	    ROS_INFO_STREAM("interval: " << INTERVAL);
    else
		ROS_WARN("Not found interval"); 

	// temp_cloud_list.resize(WINDOWS_SIZE);
	// cloud_matrix_list.resize(WINDOWS_SIZE);

//全局计数
	global_count = 0;
	frame_count = 1;

	temp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	colored_temp_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

// //初始化点云列表、点云配准rt矩阵列表
// 	for (int i=0; i< WINDOWS_SIZE; i++)
// 	{
// 		temp_cloud_list[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
// 		cloud_matrix_list[i] = Eigen::Matrix4f::Identity();
	
// 	}

	ros::Subscriber pc_sub = nh.subscribe(lidar_topic, 300, Callback);

	// Create a ROS publisher for the output point cloud
	pcPublisher = nh.advertise<sensor_msgs::PointCloud2> (fusion_pc_topic, 1);

	// Spin
	ros::spin ();
	
	return 0;
}

