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
        float RANGE;

        ros::NodeHandle n;
        ros::Publisher polar_circles_publisher;
        ros::Publisher polar_lines_publisher;
        jsk_recognition_msgs::Circle2D circle;
        jsk_recognition_msgs::Circle2DArray circle_array;
        jsk_recognition_msgs::Line line;
        jsk_recognition_msgs::LineArray line_array;
}


int main()
{
    ros::init(argc, argv, "polar_grid_viz");
    
    PolarGridViz polar_grid_viz;
    polar_grid_viz.exe();

    return 0;
}


PolarGridViz::PolarGridViz(void)
{
    circle.header = "/base_line";
    circle.x = 0.0;
    circle.y = 0.0;
    circle_array.header = "/base_link";
    circle_array.circles.resize(CIRCLE_NUM);
    line.x1 = 0.0;
    line.y1 = 0.0;
    line_array.header = "/base_link"
    lin_array.lines.resize(LINE_NUM);

    polar_circles_publisher = n.advertize<jsk_recognition_msgs::Circle2DArray>("/polar_circles");
    polar_lines_publisher = n.advertize<jsk_recognition_msgs::LineArray>("/polar_lines");
}


PolarGridViz::exe(void)
{
    ros::Rate r(100);
    while(ros::ok()){
        

        r.sleep();
        ros::spinOnce();
    }
}





