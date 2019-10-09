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
	int callback_cnt;
	int detect_cnt;
	float detect_rate;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "intersection_detected_rate_calcurator");

	IntersectionDetectRate intersection_detect_rate;
	intersection_detect_rate.execution();
}


IntersectionDetectRate::IntersectionDetectRate(void)
{
	flag_sub = n.subscribe("/intersectioin_flag", 1, &IntersectionDetectRate::intersection_flag_callback, this);
	callback_cnt = 0;
	detect_cnt = 0;
	detect_rate = 0.0;
}


void IntersectionDetectRate::execution(void)
{
	ros::spin();
}


void IntersectionDetectRate::intersection_flag_callback(const std_msgs::BoolConstPtr& msg)
{
	callback_cnt++;

	bool intersectioin_flag = msg->data;
	if(intersectioin_flag){
		detect_cnt++;
	}

	detect_rate = (float)(detect_cnt / callback_cnt);

	std::cout << "intersection detect rate = " << detect_rate << std::endl;
}


