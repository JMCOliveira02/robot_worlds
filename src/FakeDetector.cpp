#include "robot_worlds/FakeDetector.hpp"

KeypointDetector::KeypointDetector() : Node("keypoint_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {

    std::string yaml_file = "square_no_box.yaml";

    std::string feature_path;

    try {
        feature_path = ament_index_cpp::get_package_share_directory("robot_worlds") + "/feature_maps/" + yaml_file;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get package share directory: %s", e.what());
        return;
    }

    try {
        features_ = load_features(feature_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load features: %s", e.what());
        return;
    }

    for (const auto& f : features_) {
        RCLCPP_INFO(get_logger(), "Feature: %s, x: %f, y: %f, theta: %f", f.type.c_str(), f.x, f.y, f.theta);
    }
    
    corners_pub_ = this->create_publisher<robot_msgs::msg::FeatureArray>("/corner", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&KeypointDetector::checkAndPublishKeypoints, this));
}

std::vector<KeypointDetector::Feature> KeypointDetector::load_features(const std::string& yaml_file_path) {
    std::vector<KeypointDetector::Feature> features;

    YAML::Node root = YAML::LoadFile(yaml_file_path);
    if (!root["features"]) {
        throw std::runtime_error("No 'features' key found in YAML.");
    }

    for (const auto& f : root["features"]) {
        Feature feature;
        feature.type = f["type"].as<std::string>();
        feature.x = f["position"]["x"].as<double>();
        feature.y = f["position"]["y"].as<double>();
        feature.theta = f["orientation"]["theta"].as<double>();

        features.push_back(feature);
    }

    return features;
}

void KeypointDetector::checkAndPublishKeypoints() {
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Time now = this->get_clock()->now() - rclcpp::Duration::from_seconds(0.05);

    try {
        transform = tf_buffer_.lookupTransform("base_link_real", "map", now);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "Could not get robot transform: %s", ex.what());
        return;
    }

    publishTransformedCorners(transform);
    
}

void KeypointDetector::publishTransformedCorners(const geometry_msgs::msg::TransformStamped& transform) {
    robot_msgs::msg::FeatureArray corner_msg;

    int i = 0;
    for (const auto &feature : features_) {
        robot_msgs::msg::Feature corner;
        //
        // POSITION ------------------------------------
        geometry_msgs::msg::Point point;
        point.x = feature.x;
        point.y = feature.y;
        point.z = 0.0;

        geometry_msgs::msg::Point transformed_point;
        tf2::doTransform(point, transformed_point, transform);

        corner.position = transformed_point;
        // ------------------------------------ POSITION
        

        //
        // ORIENTATION ------------------------------------
        double feature_theta_radians = feature.theta * M_PI / 180.0 ;
        tf2::Quaternion q_feature;
        q_feature.setRPY(0, 0, feature_theta_radians);


        tf2::Quaternion q_transform;
        q_transform.setX(transform.transform.rotation.x);
        q_transform.setY(transform.transform.rotation.y);
        q_transform.setZ(transform.transform.rotation.z);
        q_transform.setW(transform.transform.rotation.w);

        tf2::Quaternion q = q_transform * q_feature;

    
        corner.orientation = tf2::toMsg(q);
        // ------------------------------------ ORIENTATION

        corner.position_covariance = {
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.0
        };

        // ORIENTATION COVARIANCE (only yaw matters, rest 0)
        corner.orientation_covariance = {
            0.0, 0.0,  0.0,
            0.0, 0.0,  0.0,
            0.0, 0.0,  0.05 // 0.05 rad² noise on yaw
        };

        corner_msg.features.push_back(corner);
    }

    corners_pub_->publish(corner_msg);

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeypointDetector>());
    rclcpp::shutdown();
    return 0;
}
