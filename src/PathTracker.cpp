#include "robot_worlds/PathTracker.hpp"

PathTracker::PathTracker() : Node("path_tracker"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    RCLCPP_INFO(this->get_logger(), "PathTracker Constructor!");

    real_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("real_path", 10);
    estimated_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("estimated_path", 10);

    real_path.header.frame_id = "map";
    estimated_path.header.frame_id = "map";

    update_path_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PathTracker::updatePaths, this)
    );

}

void PathTracker::updatePaths() {
    updatePathForFrame("base_footprint_real", real_path, real_path_pub_);
    updatePathForFrame("estimated_pose", estimated_path, estimated_path_pub_);

}


void PathTracker::updatePathForFrame(const std::string& frame_id, nav_msgs::msg::Path& path,
                            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub)
{
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_.lookupTransform("map", frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "tf [%s] is not available!, error: %s", frame_id.c_str(), ex.what());
        return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = tf.header;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;

    path.header.stamp = this->get_clock()->now();
    path.poses.push_back(pose);
    pub->publish(path);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathTracker>());
    rclcpp::shutdown();
    return 0;
}