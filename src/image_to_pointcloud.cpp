#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


class ImageToPointCloud
{
public:
	typedef pcl::PointXYZRGBA PointRGBA;
	typedef pcl::PointCloud<PointRGBA> CloudRGBA;
	typedef pcl::PointCloud<PointRGBA>::Ptr CloudRGBAPtr;

	ImageToPointCloud(void);

	void exe(void);
	void input_depth_image(void);
	void input_intensity_image(void);
	void input_normal_image(void);

private:
	bool depth_flag;
	bool intensity_flag;
	bool normal_flag;
	struct BGR{
		int b;
		int g;
		int r;
	};
	std::vector<float> depth_info;
	std::vector<float> intensity_info;
	std::vector<BGR> normal_info;

	ros::Publisher pc_pub;
	sensor_msgs::PointCloud2 ros_pc;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_to_pointcloud");

	ImageToPointCloud image_to_pointcloud;
	image_to_pointcloud.exe();

	return 0;
}


ImageToPointCloud::ImageToPointCloud(void)
{
	depth_flag = false;
	intensity_flag = false;
	normal_flag = false;

	ros::NodeHandle nh;
	pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/pc_from_img", 1);
}


void ImageToPointCloud::exe(void)
{
	CloudRGBAPtr pcl_pc {new CloudRGBA};
	pcl_pc.resize(131072);
	int itr = 0;
	if(depth_flag && intensity_flag && normal_flag){
		for(int row = 0; row < 64; row++){
			for(int col = 0; col < 2048; col++){
				pcl_pc->pointsi[itr].x = depth_info[itr] * cos(itr*2*M_PI/2048 - M_PI);
				pcl_pc->points[itr].y = depth_info[itr] * sin(itr*2*M_PI/2048 - M_PI);
				if(row < 32){
					pc->points[itr].z = depth_info[itr] * sin(itr*0.33333*M_PI/180);
				}else{
					pc->points[itr].z = depth_info[itr] * sin(itr*0.50*M_PI/180);
				}
				pcl_pc->points[itr].r = normal_info[itr].r;
				pcl_pc->points[itr].g = normal_info[itr].g;
				pcl_pc->points[itr].b = normal_info[itr].b;
				pcl_pc->points[itr].a = intensity_info[itr];
				itr++;
			}
		}

		pcl_pc->header.frame_id = "/velodyne";
		pcl_pc->header.stamp = ros::Time::now();
		pcl::toROSMsg(*pcl_pc, ros_pc);
		pc_pub.publish(ros_pc);
	}

	depth_flag = false;
	intensity_flag = false;
	normal_flag = false;
	depth_info.clear();
	intensity_info.clear();
	normal_info.clear();
	pcl_pc->points.clear();
}


void ImageToPointCloud::input_depth_image(void)
{
	cv::Mat src_img;
	src_img = cv::imread("/home/amsl/Desktop/taw2019/datasets/hoge/fuga.png", 0);

	if(src_img.empty()) return -1;

	for(int row = 0; row < 64; row++){
		for(int col = 0; col < 2048; col++){
			depth_info.push_back(100 * src_img[row][col]);
		}
	}

	depth_flag = true;
}


void ImageToPointCloud::input_intensity_image(void)
{
	cv::Mat src_img;
	src_img = cv::imread("/home/amsl/Desktop/taw2019/datasets/hoge/fuga.png", 0);

	if(src_img.empty()) return -1;

	for(int row = 0; row < 64; row++){
		for(int col = 0; col < 2048; col++){
			intensity_info.push_back(src_img[row][col]);
		}
	}

	intensity_flag = true;
}


void ImageToPointCloud::input_normal_image(void)
{
	cv::Mat src_img;
	src_img = cv::imread("/home/amsl/Desktop/taw2019/datasets/hoge/fuga.png", 1);

	if(src_img.empty()) return -1;

	cv::Mat channels[3], red, blue, green;
	cv::split(src_img, channels);
	blue = channels[0];
	green = channels[1];
	red = channels[2];

	for(int row = 0; row < 64; row++){
		for(int col = 0; col < 2048; col++){
			BGR bgr;
			bgr.b = blue[row][col];
			bgr.g = green[row][col];
			bgr.r = red[row][col];
			normal_info.push_back(bgr);
		}
	}

	normal_flag = true;
}

