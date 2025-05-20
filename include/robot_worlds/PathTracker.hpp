#ifndef PATH_TRACKER_HPP
#define PATH_TRACKER_HPP

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
#include <nav_msgs/msg/path.hpp>

class PathTracker : public rclcpp::Node {
public: 
    PathTracker();
private:

    // Path messages
    nav_msgs::msg::Path real_path;
    nav_msgs::msg::Path estimated_path;

    // Real and estimated path publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr real_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr estimated_path_pub_;

    // Pose subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    // Timer for paths update
    rclcpp::TimerBase::SharedPtr update_path_timer_;

    // Buffer and tf listener for tf tracking
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Methods to update the paths
    void updatePaths();
    void updatePathForFrame(const std::string& frame_id, nav_msgs::msg::Path& path,
                            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub);
    void updatePathCallback(const std::string& topic, nav_msgs::msg::Path& path,
                            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub);

};

#endif //PATH_TRACKER_HPP