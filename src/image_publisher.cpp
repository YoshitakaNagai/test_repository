#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


class ImagePublisher
{
public:
	ImagePublisher();
	void exe(void);

private:
	sensor_msgs::ImagePtr img_msg;
	cv::Mat img;
	image_transport::Publisher image_pub;
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "segmented_image_publisher");

	ImagePublisher image_publisher;
	image_publisher.exe();

	return 0;
}


ImagePublisher::ImagePublisher(void)
{
	ros::NodeHandle nh("~");
	image_transport::ImageTransport it(nh);
	img = cv::imread("/home/amsl/ros_catkin_ws/src/test_repository/src/dataset/robosym2020_picture/segmented_img/20180804132428135880_seg.png", 1);
	image_pub = it.advertise("/segmented_image", 10);
}


void ImagePublisher::exe(void)
{
	ros::Rate r(100);
	img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	while(ros::ok()){
		image_pub.publish(img_msg);
		std::cout << "pub image" << std::endl;

	}
	ros::spinOnce();
	r.sleep();
}










