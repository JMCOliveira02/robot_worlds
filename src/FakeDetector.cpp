#include "robot_worlds/FakeDetector.hpp"

KeypointDetector::KeypointDetector() : Node("keypoint_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {

    std::string yaml_file = "square_two_boxes.yaml";

    std::string feature_path;

    try {
        feature_path = ament_index_cpp::get_package_share_directory("robot_worlds") + "/feature_maps/" + yaml_file;
        std::cout<<"Feature path: "<<feature_path<<std::endl;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get package share directory: %s", e.what());
        return;
    }

    try {
        map_features::MapLoader loader;
        RCLCPP_INFO(this->get_logger(), "Loading features from %s", feature_path.c_str());
        loader.loadToGlobalMap(feature_path);
        global_features_ =map_features::MapLoader::getGlobalFeatureMap();


    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load features: %s from %s", e.what(), feature_path.c_str());
        return;
    }
    
    corners_pub_ = this->create_publisher<robot_msgs::msg::FeatureArray>("/corner", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&KeypointDetector::checkAndPublishKeypoints, this));
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

double KeypointDetector::computeSensorNoise(double distance) {
    double min_noise = 0.04;
    double max_noise = 0.4;  
    double max_distance = 1.5; 

    // Quadratic noise model (better fit for real-world LIDAR)
    //double noise = min_noise + (max_noise - min_noise) * (distance * distance / (max_distance * max_distance));

    //linear noise model
    double noise = min_noise + (max_noise - min_noise) * (distance / max_distance);

    //std::cout<<"Noise: "<<noise<<std::endl;

    return std::min(std::max(noise, min_noise), max_noise);
}


void KeypointDetector::publishTransformedCorners(const geometry_msgs::msg::TransformStamped& transform) {
    robot_msgs::msg::FeatureArray corner_msg;

    int i = 0;
    for (const auto &feature : global_features_) {
        if (feature->type != "corner") continue;
    
        auto corner_map = std::dynamic_pointer_cast<map_features::FeatureCorner>(feature);
     
        robot_msgs::msg::Feature corner;
        //
        // POSITION ------------------------------------
        geometry_msgs::msg::Point point;
        point.x = corner_map->x;
        point.y = corner_map->y;
        point.z = 0.0;

        geometry_msgs::msg::Point transformed_point;
        tf2::doTransform(point, transformed_point, transform);

        corner.position = transformed_point;
        // ------------------------------------ POSITION
        

        //
        // ORIENTATION ------------------------------------
        double feature_theta_radians = corner_map->theta * M_PI / 180.0 ;
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

        double distance = std::hypot(transformed_point.x, transformed_point.y);
        double noise = computeSensorNoise(distance);

        corner.position_covariance = {
            noise*noise, 0.0,  0.0,
            0.0,  noise*noise, 0.0,
            0.0,  0.0,  0.0
        };

        // ORIENTATION COVARIANCE (only yaw matters, rest 0)
        corner.orientation_covariance = {
            0.0, 0.0,  0.0,
            0.0, 0.0,  0.0,
            0.0, 0.0,  0.1 // 0.05 rad² noise on yaw
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
