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

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>

#include <pcl/point_types.h>  
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>

#include <pcl/filters/passthrough.h>
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
Eigen::Matrix4f Ti, Ta, TempT;

ros::Publisher pcPublisher;   

int global_count, frame_count;

pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr fused_cloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_temp_cloud;
std::vector<Eigen::Matrix4f> T_list;

std::string pkg_loc = ros::package::getPath("pcl_reg");


Eigen::Array3f Rotation2Euler(const Eigen::Matrix3f& rot)
{
    Eigen::Vector3f angles = rot.eulerAngles(2,1,0);// 2代表Z轴，1代表Y轴，0代表X轴
    double x_angle = angles[2];
    double y_angle = angles[1];
    double z_angle = angles[0];
    
    if(y_angle<-M_PI/2 || y_angle>M_PI/2)
	{
		x_angle += M_PI;
		if(x_angle < -M_PI)      x_angle += 2*M_PI;
		else if(x_angle > M_PI)  x_angle -= 2*M_PI;
		
		y_angle = M_PI - y_angle;
		if(y_angle > M_PI) y_angle -= 2*M_PI;
			
		z_angle -= M_PI;
	}
    
    return Eigen::Array3f(x_angle, y_angle, z_angle) * 57.29578; //转为角度值
}

Eigen::Quaternionf addQ(Eigen::Quaternionf a, Eigen::Quaternionf b)
{
	Eigen::Quaternionf retval;
	if(a.x()*b.x() + a.y()*b.y() + a.z()*b.z() + a.w()*b.w() < 0.0)
	{
		b.x() = -b.x();
		b.y() = -b.y();
		b.z() = -b.z();
		b.w() = -b.w();
	}
	retval.x() = a.x() + b.x();
	retval.y() = a.y() + b.y();
	retval.z() = a.z() + b.z();
	retval.w() = a.w() + b.w();
	return retval;
}

void average(Eigen::Matrix4f &T1, Eigen::Matrix4f &T2, Eigen::Matrix4f &Tout) //T为平均后的值
{

	Eigen::Matrix3f rotation1, rotation2;
	Eigen::Vector3f translation1, translation2;
	Eigen::Quaternionf rotation_sum = Eigen::Quaternionf(0.0, 0.0, 0.0, 0.0);
	Eigen::Vector3f translation_avg(0, 0, 0);

	rotation1 = T1.topLeftCorner<3, 3>();
	translation1 = T1.col(3).head(3);

	rotation2 = T2.topLeftCorner<3, 3>();
	translation2 = T2.col(3).head(3);


	// averaging translation and rotation
	translation_avg = (translation1 + translation2)/2;  

	Eigen::Quaternionf temp_q1(rotation1);
	Eigen::Quaternionf temp_q2(rotation2);
	rotation_sum = addQ(temp_q1, temp_q2);

	rotation_sum.x() = rotation_sum.x() / 2;
	rotation_sum.y() = rotation_sum.y() / 2;
	rotation_sum.z() = rotation_sum.z() / 2;
	rotation_sum.w() = rotation_sum.w() / 2;
	double mag = sqrt(rotation_sum.x()*rotation_sum.x() +
					rotation_sum.y()*rotation_sum.y() +
					rotation_sum.z()*rotation_sum.z() +
					rotation_sum.w()*rotation_sum.w());
	rotation_sum.x() = rotation_sum.x()/mag;
	rotation_sum.y() = rotation_sum.y()/mag;
	rotation_sum.z() = rotation_sum.z()/mag;
	rotation_sum.w() = rotation_sum.w()/mag;

	Eigen::Matrix3f rotation_avg = rotation_sum.toRotationMatrix();
	// Eigen::Matrix4f T; 
	Tout.setIdentity(4,4);    //单位矩阵
	Tout.topLeftCorner(3, 3) = rotation_avg;
	Tout.col(3).head(3) = translation_avg;
}

//过滤
void cloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filtered, bool downsample = true)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr sor_cloud (new pcl::PointCloud<pcl::PointXYZI>), index_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	
	//removing outlier points
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud(cloud);
	pcl::PointIndices inlier;
	for (size_t i=0; i < cloud->points.size(); ++i)
	{
		if   (cloud->points[i].x > -20  && cloud->points[i].x < 20
		 &&  cloud->points[i].y > -20  && cloud->points[i].y < 20  
		 &&  cloud->points[i].z > -2  && cloud->points[i].z < 5 )
		 		//  &&  !(sqrt(pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2) + pow(cloud->points[i].z, 2)) < 1) 
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
		pcl::VoxelGrid<pcl::PointXYZI> sor;
		sor.setInputCloud(index_cloud);
		sor.setLeafSize(0.05f, 0.05f, 0.05f);
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

	pcl::SACSegmentation<pcl::PointXYZI> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	// you can modify the parameter below
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.4);
	// extract ground
	pcl::ExtractIndices<pcl::PointXYZI> extractor;

	
	for (int i =0; i< 3 ; ++i)
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


void coloredPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc1, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc2)
{
	int r,g,b;
	r = random(0, 255);  
	g = random(0, 255);
	b = random(0, 255);

	// addRGB(pc1, *pc2, r,g,b);
	for (size_t i=0;i<pc1->points.size();i++)
	{

		pcl::PointXYZRGB p;
		p.x = pc1->at(i).x;
		p.y = pc1->at(i).y;
		p.z = pc1->at(i).z;
		p.r = r;
		p.g = g;
		p.b = b;
		pc2->push_back(p);
	}
	pc2->width = 1;
	pc2->height = pc1->points.size();

}

//----------------加速的ndt
void pclomp_ndtRegistration(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI>::Ptr dst, Eigen::Matrix4f &transform)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr out (new pcl::PointCloud<pcl::PointXYZI>);

	pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
	ndt_omp->setResolution(1);
	ndt_omp->setTransformationEpsilon (1e-6);   //为终止条件设置最小转换差异
	ndt_omp->setStepSize (0.2);    //为more-thuente线搜索设置最大步长
	ndt_omp->setMaximumIterations (200);
	ndt_omp->setNumThreads(16);
	ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);  //pclomp::KDTREE   pclomp::DIRECT1

	ndt_omp->setInputSource (src);  //源点云
	// Setting point cloud to be aligned to.
	ndt_omp->setInputTarget (dst);  //目标点云
	ndt_omp->align(*out);// not use init_guess

	// colored_PCD_Publish(dst, output_cloud, pcPublisher);

	// std::cout << "has converged:  " << ndt_omp->hasConverged() << "  score: " << ndt_omp->getFitnessScore() << std::endl;
	// std::cout << ndt_omp->getFinalTransformation() << std::endl;
    transform = ndt_omp->getFinalTransformation();
	TempT =  ndt_omp->getFinalTransformation();
	// Ti = ndt_omp->getFinalTransformation() * Ti;
}

void Callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg)   //不订阅camerainfo的回调函数
{
//全局计数
	global_count += 1;	
	// ROS_INFO_STREAM(" The " << global_count << " frame\n");
	// ROS_INFO_STREAM("PointCloud scan received at " << pc_msg->header.stamp << "ns");

	sensor_msgs::PointCloud2 msg_clouds;
	pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*pc_msg, *input_cloud);
	Eigen::Matrix4f Ts,Tf,Tb;

	cloudFilter(input_cloud, input_cloud, false);  //过滤

	if ( temp_cloud->points.size() == 0 )  
	{
		temp_cloud = input_cloud;
		fused_cloud = input_cloud;
		coloredPoints(temp_cloud, colored_temp_cloud);	
	}

	// clock_t startTime=clock();
	pclomp_ndtRegistration(input_cloud, temp_cloud, Ts);  //点云配准， 外参矩阵
	// clock_t pcTime=clock();
	// std::cout<<"\n 点云配准时间: "<<(double)(pcTime - startTime)/(double)CLOCKS_PER_SEC<<" s"<< std::endl;	

	temp_cloud = input_cloud;
	Ti = Ti* Ts;

	// std::cout << "Rotation2Euler(Ti.topLeftCorner<3, 3>()): \n" << Rotation2Euler(Ti.topLeftCorner<3, 3>()).transpose() << std::endl;

	if (global_count % INTERVAL == 0)
	{
		ROS_INFO_STREAM(" The " << global_count << " frame\n");
		frame_count +=1;

		pcl::transformPointCloud(*input_cloud, *output_cloud, Ti);
		coloredPoints(output_cloud, colored_output_cloud);  

		// *fused_cloud += *output_cloud;
		*colored_temp_cloud += *colored_output_cloud;
		// cloudFilter(fused_cloud, fused_cloud, true);  //过滤

		if ( frame_count == FRAMENUMBER)
		{
			std::stringstream ss;
			ss << pkg_loc+"final.pcd";
			std::cout << ss.str();
			pcl::io::savePCDFile(ss.str(), *fused_cloud, true);
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
	ros::init (argc, argv, "trans_and_project");
	ros::NodeHandle nh("~");

    std::string lidar_topic, fusion_pc_topic;
	Ti.setIdentity(4,4);
	Ta.setIdentity(4,4);
	TempT.setIdentity(4,4);

	if (nh.param<std::string>("lidar_topic", lidar_topic, "/velodyne_points"))  //订阅点云话题
	    ROS_INFO_STREAM("lidar_topic: " << lidar_topic);
    else
		ROS_WARN("Not found lidar_topic"); 



	if (nh.param<std::string>("fusion_pc_topic", fusion_pc_topic, "/fusion_points"))   
	    ROS_INFO_STREAM("fusion_pc_topic: " << fusion_pc_topic);
    else
		ROS_WARN("Not found fusion_pc_topic"); 

	if (nh.param<int>("framenumber", FRAMENUMBER, 10))   
	    ROS_INFO_STREAM("framenumber: " << FRAMENUMBER);
    else
		ROS_WARN("Not found framenumber"); 

	if (nh.param<int>("interval", INTERVAL, 5))  
	    ROS_INFO_STREAM("interval: " << INTERVAL);
    else
		ROS_WARN("Not found interval"); 

//全局计数
	global_count = 0;
	frame_count = 1;

	temp_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
	fused_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
	colored_temp_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	ros::Subscriber pc_sub = nh.subscribe(lidar_topic, 1, Callback);

	// Create a ROS publisher for the output point cloud
	pcPublisher = nh.advertise<sensor_msgs::PointCloud2> (fusion_pc_topic, 1);

	// Spin
	ros::spin ();
	
	return 0;
}

