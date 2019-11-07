#include <ros/ros.h>
#include "jsk_recognition_msgs/Circle2D.h"
#include "jsk_recognition_msgs/Circle2DArray.h"
#include "jsk_recognition_msgs/Line.h"
#include "jsk_recognition_msgs/LineArray.h"


class PolarGridViz
{
    public:
        PolarGridViz(void);
        void exe(void);

    private:
        const int CIRCLE_NUM = 20;
        const int LINE_NUM = 120;
        const double MAX_RANGE = 20.0;
        double RANGE;
        double dTheta = 2 * M_PI / (double)LINE_NUM;

        ros::NodeHandle n;
        ros::Publisher polar_circles_publisher;
        ros::Publisher polar_lines_publisher;
        jsk_recognition_msgs::Circle2D circle;
        jsk_recognition_msgs::Circle2DArray circle_array;
        jsk_recognition_msgs::Line line;
        jsk_recognition_msgs::LineArray line_array;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "polar_grid_viz");
    
    PolarGridViz polar_grid_viz;
    polar_grid_viz.exe();

    return 0;
}


PolarGridViz::PolarGridViz(void)
{
    circle_array.header.frame_id = "/velodyne";
    circle.header.stamp = ros::Time::now();
    circle_array.circles.resize(CIRCLE_NUM);
    line_array.header.frame_id = "/velodyne";
	line_array.header.stamp = ros::Time::now();
    line_array.lines.resize(LINE_NUM);

    polar_circles_publisher = n.advertise<jsk_recognition_msgs::Circle2DArray>("/polar_circles", 10);
    polar_lines_publisher = n.advertise<jsk_recognition_msgs::LineArray>("/polar_lines", 10);
}


void PolarGridViz::exe(void)
{
    ros::Rate r(100);
    while(ros::ok()){
        for(size_t n = 0; n < (size_t)CIRCLE_NUM; n++){
    		circle_array.circles[n].header.frame_id = "/velodyne";
    		circle_array.circles[n].header.stamp = ros::Time::now();
            circle_array.circles[n].x = 0.0;
            circle_array.circles[n].y = 0.0;
            circle_array.circles[n].radius = (double)n;
        }

        double theta = 0.0;
        for(size_t n = 0; n < (size_t)LINE_NUM; n++){
    		/* line_array.lines[n].header.frame_id = "/velodyne"; */
			/* line_array.lines[n].header.stamp = ros::Time::now(); */
            line_array.lines[n].x2 = MAX_RANGE * cos(theta);
            line_array.lines[n].y2 = MAX_RANGE * sin(theta);
            theta += dTheta;
        }
        
        polar_circles_publisher.publish(circle_array);
        polar_lines_publisher.publish(line_array);

        r.sleep();
        ros::spinOnce();
    }
}
