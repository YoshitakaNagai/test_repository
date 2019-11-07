#include <omp.h>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

class PointClassChecker
{
public:
    typedef pcl::PointXYZRGB PointType ;
    typedef pcl::PointCloud<PointType> PointCloudType;
    typedef PointCloudType::Ptr PointCloudTypePtr;

    PointClassChecker(void);

    void callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
    bool is_in_extraction_classes(int, int, int);
    void process(void);

private:
    double MAX_DISTANCE;
    std::string LABELS_PATH;
    std::string EXTRACTION_CLASSES;
    std::vector<std::string> EXTRACTION_CLASSES_LIST;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> sensor_fusion_sync_subs;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
    message_filters::Synchronizer<sensor_fusion_sync_subs> sensor_fusion_sync;

    ros::Publisher image_pub;
    ros::Publisher pc_pub;
    ros::Publisher semantic_cloud_pub;
    ros::Publisher projection_semantic_image_pub;

    tf::TransformListener listener;
    Eigen::Affine3d transform;
    typedef std::tuple<int, int, int> ColorTuple;
    std::map<ColorTuple, std::string> color_with_class;
};



PointClassChecker::PointClassChecker(void)
    : local_nh("~"),
      image_sub(nh, "/segmented_image", 10), camera_info_sub(nh, "/camera_info", 10), pc_sub(nh, "/pointcloud", 10), sensor_fusion_sync(sensor_fusion_sync_subs(10), image_sub, camera_info_sub, pc_sub)
{
    image_pub = nh.advertise<sensor_msgs::Image>("/projection/raw", 1);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/colored/raw", 1);
    semantic_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/colored/semantic", 1);
    projection_semantic_image_pub = nh.advertise<sensor_msgs::Image>("/projection/semantic", 1);
    sensor_fusion_sync.registerCallback(boost::bind(&SemanticSegmentationWithPointCloudIntegrator::callback, this, _1, _2, _3));

    local_nh.param("MAX_DISTANCE", MAX_DISTANCE, {20.0});
    local_nh.param("LABELS_PATH", LABELS_PATH, {""});
    local_nh.param("EXTRACTION_CLASSES", EXTRACTION_CLASSES, {"ground, road, sidewalk, terrain"});

    std::cout << "=== semantic_segmentation_with_point_cloud_integrator ===" << std::endl;
    std::cout << "MAX_DISTANCE: " << MAX_DISTANCE << std::endl;
    std::cout << "LABELS_PATH: " << LABELS_PATH << std::endl;
    std::cout << "EXTRACTION_CLASSES: " << std::endl;

    // remove space
    size_t pos;
    while((pos = EXTRACTION_CLASSES.find_first_of(" 　\t")) != std::string::npos){
        EXTRACTION_CLASSES.erase(pos, 1);
    }
    // separate by ","
    while(1){
        static auto offset = std::string::size_type(0);
        auto pos = EXTRACTION_CLASSES.find(",", offset);
        if(pos == std::string::npos){
            EXTRACTION_CLASSES_LIST.push_back(EXTRACTION_CLASSES.substr(offset));
            break;
        }
        EXTRACTION_CLASSES_LIST.push_back(EXTRACTION_CLASSES.substr(offset, pos - offset));
        offset = pos + std::string(",").length();
    }
    for(const auto& str : EXTRACTION_CLASSES_LIST){
        std::cout << str << std::endl;
    }

    // load json
    boost::property_tree::ptree ptree;
    boost::property_tree::read_json(LABELS_PATH, ptree);

    BOOST_FOREACH(boost::property_tree::ptree::value_type &child, ptree.get_child("labels"))
    {
        std::string class_name = "";
        int red = 0;
        int green = 0;
        int blue = 0;
        const boost::property_tree::ptree& labels = child.second;
        if(boost::optional<std::string> label = labels.get_optional<std::string>("label")) {
            std::cout << "label : " << label.get() << std::endl;
            class_name = label.get();
        }else{
            std::cout << "label is nothing" << std::endl;
            exit(-1);
        }
        if(std::find(EXTRACTION_CLASSES_LIST.begin(), EXTRACTION_CLASSES_LIST.end(), class_name) == EXTRACTION_CLASSES_LIST.end()){
            std::cout << "skipped" << std::endl;
            continue;
        }
        const boost::property_tree::ptree& color = child.second.get_child("color");
        if(boost::optional<int> r = color.get_optional<int>("r")){
            std::cout << "r : " << r.get() << std::endl;
            red = r.get();
        }else{
            std::cout << "color.r is nothing" << std::endl;
            exit(-1);
        }
        if(boost::optional<int> g = color.get_optional<int>("g")){
            std::cout << "g : " << g.get() << std::endl;
            green = g.get();
        }else{
            std::cout << "color.g is nothing" << std::endl;
            exit(-1);
        }
        if(boost::optional<int> b = color.get_optional<int>("b")){
            std::cout << "b : " << b.get() << std::endl;
            blue = b.get();
        }else{
            std::cout << "color.b is nothing" << std::endl;
            exit(-1);
        }
        std::cout << red << ", " << green << ", " << blue << ", " << class_name << std::endl;;
        color_with_class[std::make_tuple(red, green, blue)] = class_name;
    }
    std::cout << "json was loaded" << std::endl;
    std::cout << "color with class:" << std::endl;
    std::cout << "size: " << color_with_class.size() << std::endl;
    for(const auto& value : color_with_class){
        std::cout << "red: " << std::get<0>(value.first) << ", green:" << std::get<1>(value.first) << ", blue: " << std::get<2>(value.first) << ", class: " << value.second << std::endl;
    }
    std::cout << "waiting for data..." << std::endl;
}

void PointClassChecker::callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info)
{
}


bool PointClassChecker::is_in_extraction_classes(int r, int g, int b)
{
    auto color_tuple = std::make_tuple(r, g, b);
    for(const auto& value : color_with_class){
        if(color_tuple == value.first){
            return true;
        }
    }
    return false;
}

void PointClassChecker::process(void)
{
    ros::spin();
}
