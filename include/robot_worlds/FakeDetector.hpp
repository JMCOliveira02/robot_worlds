#ifndef FAKE_DETECTOR
#define FAKE_DETECTOR

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <random>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <robot_msgs/msg/feature.hpp>
#include <robot_msgs/msg/feature_array.hpp>
#include <robot_worlds/FeatureStruct.hpp>
#include <robot_worlds/MapLoader.hpp>


class KeypointDetector : public rclcpp::Node {
public:
    KeypointDetector();

private:
    std::string map_features_;
    
    std::vector<map_features::FeaturePtr> global_features_;

    std::vector<std::pair<double, double>> keypoints_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Random number generator
    std::default_random_engine generator_;

    rclcpp::Publisher<robot_msgs::msg::FeatureArray>::SharedPtr feature_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr feature_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr keypoints_markers_pub_;

    void checkAndPublishKeypoints();

    rclcpp::TimerBase::SharedPtr timer_;
    
    void publishTransformedFeatures(const geometry_msgs::msg::TransformStamped& transform);

    double computeSensorNoise(double distance) ;

};

#endif // FAKE_DETECTOR
