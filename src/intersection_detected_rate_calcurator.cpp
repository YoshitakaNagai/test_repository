#include <ros/ros.h>
#include <std_msgs/Bool.h>


class IntersectionDetectRate
{
public:
	IntersectionDetectRate(void);
	void execution(void);
	void intersection_flag_callback(const std_msgs::BoolConstPtr&);

private:
	ros::NodeHandle n;
	ros::Subscriber flag_sub;
	bool callback_flag;
	bool intersection_flag;
	int callback_cnt;
	int detect_cnt;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "intersection_detected_rate_calcurator");

	IntersectionDetectRate intersection_detect_rate;
	intersection_detect_rate.execution();
}


IntersectionDetectRate::IntersectionDetectRate(void)
{
	flag_sub = n.subscribe("/intersection_flag", 1, &IntersectionDetectRate::intersection_flag_callback, this);
	callback_flag = false;
}


void IntersectionDetectRate::execution(void)
{
	intersection_flag = false;
	callback_cnt = 0;
	detect_cnt = 0;
    ros::Rate loop_rate(1000);
    
	while(ros::ok()){
		if(callback_flag){
			callback_cnt++;
			if(intersection_flag){
				detect_cnt++;
			}
			std::cout << "callback_cnt : " << callback_cnt << std::endl;
			std::cout << "detect_cnt : " << detect_cnt << std::endl;

			callback_flag = false;
		}
		ros::spinOnce();
		// loop_rate.sleep();
	}
}


void IntersectionDetectRate::intersection_flag_callback(const std_msgs::BoolConstPtr& msg)
{
	intersection_flag = msg->data;
	callback_flag = true;
}


