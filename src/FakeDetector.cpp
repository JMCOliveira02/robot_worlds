#include "robot_worlds/FakeDetector.hpp"

KeypointDetector::KeypointDetector() : Node("keypoint_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {

    // Retrieve the map_features parameter passed from the launch file
    this->declare_parameter("map_features", std::string(""));
    this->get_parameter("map_features", map_features_);

    try {
        map_features::MapLoader loader;
        RCLCPP_INFO(this->get_logger(), "Loading features from %s", map_features_.c_str());
        loader.loadToGlobalMap(map_features_);
        global_features_ = map_features::MapLoader::getGlobalFeatureMap();


    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load features: %s from %s", e.what(), map_features_.c_str());
        return;
    }
    
    feature_pub_ = this->create_publisher<robot_msgs::msg::FeatureArray>("/features", 10);

    feature_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/feature_markers", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&KeypointDetector::checkAndPublishKeypoints, this));
}

void KeypointDetector::checkAndPublishKeypoints() {
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Time now = this->get_clock()->now() - rclcpp::Duration::from_seconds(0.05);

    try {
        transform = tf_buffer_.lookupTransform("base_footprint_real", "map", now);

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "Could not get robot transform: %s", ex.what());
        return;
    }

    publishTransformedFeatures(transform);

    
}

double KeypointDetector::computeSensorNoise(double distance) {

    return distance * 0.15; 
}



void KeypointDetector::publishTransformedFeatures(const geometry_msgs::msg::TransformStamped& transform) {
    robot_msgs::msg::FeatureArray feature_array_msg;
    visualization_msgs::msg::MarkerArray marker_array;

    //RCLCPP_INFO(get_logger(), "Entering publish features");
    int i = 0; 
    for (const auto &feature : global_features_) {
        std::shared_ptr<map_features::Feature> feature_map;
        if(feature->type == "corner"){
            feature_map = std::dynamic_pointer_cast<map_features::FeatureCorner>(feature);
        }
        else{
            feature_map = std::dynamic_pointer_cast<map_features::FeatureObject>(feature);

        }

        if (!feature_map) {
            RCLCPP_WARN(get_logger(), "Failed to cast feature");
            continue;
        }

        //RCLCPP_INFO(get_logger(), "Feature: %s", feature_map->type.c_str());
     
        robot_msgs::msg::Feature feature_msg;
        //
        // POSITION ------------------------------------
        geometry_msgs::msg::Point point;
        point.x = feature_map->x;
        point.y = feature_map->y;
        point.z = 0.0;

        geometry_msgs::msg::Point transformed_point;
        tf2::doTransform(point, transformed_point, transform);

        feature_msg.position = transformed_point;

        RCLCPP_INFO(get_logger(), "Transformed point: (%.3f, %.3f, %.3f)", transformed_point.x, transformed_point.y, transformed_point.z);
        // ------------------------------------ POSITION
        

        //
        // ORIENTATION ------------------------------------
        double feature_theta_radians = feature_map->theta * M_PI / 180.0 ;
        tf2::Quaternion q_feature;
        q_feature.setRPY(0, 0, feature_theta_radians);


        tf2::Quaternion q_transform;
        q_transform.setX(transform.transform.rotation.x);
        q_transform.setY(transform.transform.rotation.y);
        q_transform.setZ(transform.transform.rotation.z);
        q_transform.setW(transform.transform.rotation.w);

        tf2::Quaternion q = q_transform * q_feature;


    
        feature_msg.orientation = tf2::toMsg(q);
        // ------------------------------------ ORIENTATION

        double distance = std::hypot(transformed_point.x, transformed_point.y);
        double noise = computeSensorNoise(distance);

        feature_msg.position_covariance = {
            noise*noise, 0.0,  0.0,
            0.0,  noise*noise, 0.0,
            0.0,  0.0,  0.0
        };

        // ORIENTATION COVARIANCE (only yaw matters, rest 0)
        feature_msg.orientation_covariance = {
            0.0, 0.0,  0.0,
            0.0, 0.0,  0.0,
            0.0, 0.0,  0.05*0.05
        };

        feature_msg.type = feature_map->type;

        feature_msg.confidence = 0.01;

        feature_array_msg.features.push_back(feature_msg);

        // MARKER -----------------------------------
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_footprint_real";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "feature";
        marker.id = i++;

        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = transformed_point.x;
        marker.pose.position.y = transformed_point.y;
        marker.pose.position.z = transformed_point.z;

        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.2;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

        marker_array.markers.push_back(marker);

    }

    feature_pub_->publish(feature_array_msg);
    feature_markers_pub_->publish(marker_array);

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeypointDetector>());
    rclcpp::shutdown();
    return 0;
}
