#include "robot_worlds/RobotController.hpp"

#define TIME_STEP 32
#define HALF_DISTANCE_BETWEEN_WHEELS 0.165 //0.045
#define WHEEL_RADIUS 0.094//0.024


namespace robot_controller {

bool checkMapOverlap(double x, double y){
  return false;
}

WbNodeRef robot_node = wb_supervisor_node_get_from_def("ROBOT_1");

void RobotController::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {  

  
  right_motor = wb_robot_get_device("right wheel motor");
  left_motor = wb_robot_get_device("left wheel motor");

  right_encoder = wb_robot_get_device("right wheel encoder");
  left_encoder = wb_robot_get_device("left wheel encoder");

  lidar2D = wb_robot_get_device("lidar2D");
  wb_lidar_enable(lidar2D, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar2D);

  wb_position_sensor_enable(right_encoder, TIME_STEP);
  wb_position_sensor_enable(left_encoder, TIME_STEP);


  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);

  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);


  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        this->cmd_vel_msg.linear = msg->linear;
        this->cmd_vel_msg.angular = msg->angular;
      }
  );

  tf_broadcaster_relative = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  tf_broadcaster_real = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));

  est_x = 0.0;
  est_y = 0.0;
  est_theta = 0.0;
  last_left_wheel_pos = 0.0;
  last_right_wheel_pos = 0.0;


  this->node_ = node;

}

void RobotController::step() {

  #pragma region InitializePosition
  
  double random_x = 0.0;
  double random_y = 0.0;
  double random_theta = 0.0;
  bool randomSpawn = false;
  if(false)
  {
    if(randomSpawn)
    {  
      unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
      generator_.seed(seed);
      std::uniform_real_distribution<double> xy_dist(-0.65, 0.65);
      std::uniform_real_distribution<double> theta_dist(-M_PI, M_PI);
      
      bool overlap = true;
      while(overlap)
      {
        random_x = xy_dist(generator_);
        random_y = xy_dist(generator_);
        overlap = checkMapOverlap(random_x, random_y);
      }
      random_theta = theta_dist(generator_);
    }
    std::cout << "Initializing position: x " <<  random_x <<" y: " << random_y<< std::endl;
    std::cout << "Initializing orientation: " << random_theta << std::endl;
    double init_position[3] = {random_x, random_y, 0.0};
    double init_orientation[4] = {0.0, 0.0, 1.0, random_theta};

    WbFieldRef translation_field = wb_supervisor_node_get_field(robot_node, "translation");
    WbFieldRef rotation_field = wb_supervisor_node_get_field(robot_node, "rotation");

    wb_supervisor_field_set_sf_vec3f(translation_field, init_position);
    wb_supervisor_field_set_sf_rotation(rotation_field, init_orientation);

  }

  
  #pragma endregion InitializePosition

  #pragma region RealPositionBroadcast

  const double *position = wb_supervisor_node_get_position(robot_node);
  const double *orientation = wb_supervisor_node_get_orientation(robot_node);

  tf2::Matrix3x3 mat(
    orientation[0], orientation[1], orientation[2],
    orientation[3], orientation[4], orientation[5],
    orientation[6], orientation[7], orientation[8]);

  tf2::Quaternion q;
  mat.getRotation(q);


  geometry_msgs::msg::TransformStamped tf_real;
  tf_real.header.stamp = node_->get_clock()->now();
  tf_real.header.frame_id = "map";
  tf_real.child_frame_id = "base_footprint_real";
  tf_real.transform.translation.x = position[0];
  tf_real.transform.translation.y = position[1];
  tf_real.transform.translation.z = position[2];
  tf_real.transform.rotation.x = q.x();
  tf_real.transform.rotation.y = q.y();
  tf_real.transform.rotation.z = q.z();
  tf_real.transform.rotation.w = q.w();
  tf_broadcaster_real->sendTransform(tf_real);
  #pragma endregion RealPositionBroadcast

  #pragma region EstimatedPositionBroadcast
  
  left_wheel_position = wb_position_sensor_get_value(left_encoder);
  right_wheel_position = wb_position_sensor_get_value(right_encoder);

  if(first_update){
    first_update = false;
  }
  else{
  
    double delta_left_wheel = left_wheel_position - last_left_wheel_pos;
    double delta_right_wheel = right_wheel_position - last_right_wheel_pos;

    //std::cout << "LW_Pos: " << left_wheel_position << " -- RW_Pos: " << right_wheel_position << std::endl;


    double delta_distance = (delta_left_wheel + delta_right_wheel) * WHEEL_RADIUS / 2.0 ;
    double delta_theta = (delta_right_wheel - delta_left_wheel) * WHEEL_RADIUS / (2 * HALF_DISTANCE_BETWEEN_WHEELS);

    last_left_wheel_pos = left_wheel_position;
    last_right_wheel_pos = right_wheel_position;

    double updated_theta = est_theta + delta_theta / 2.0;
    if(updated_theta > M_PI) updated_theta -= 2 * M_PI;
    if(updated_theta < -M_PI) updated_theta += 2 * M_PI;

    
    est_x += delta_distance * std::cos(updated_theta);
    est_y += delta_distance * std::sin(updated_theta);
    est_theta += delta_theta;
    if(est_theta > M_PI) est_theta -= 2 * M_PI;
    if(est_theta < -M_PI) est_theta += 2 * M_PI;
    //std::cout << "X: " << est_x << " -- Y: " << est_y << std::endl;
    
    tf2::Quaternion est_q;
    est_q.setRPY(0, 0, est_theta);

    
    
    geometry_msgs::msg::TransformStamped tf_relative;
    tf_relative.header.stamp = node_->get_clock()->now();
    tf_relative.header.frame_id = "odom";
    tf_relative.child_frame_id = "base_footprint";
    tf_relative.transform.translation.x = est_x;
    tf_relative.transform.translation.y = est_y;
    tf_relative.transform.translation.z = 0.0;
    tf_relative.transform.rotation.x = est_q.x();
    tf_relative.transform.rotation.y = est_q.y();
    tf_relative.transform.rotation.z = est_q.z();
    tf_relative.transform.rotation.w = est_q.w();
    tf_broadcaster_relative->sendTransform(tf_relative);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = node_->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = est_x;
    odom.pose.pose.position.y = est_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = est_q.x();
    odom.pose.pose.orientation.y = est_q.y();
    odom.pose.pose.orientation.z = est_q.z();
    odom.pose.pose.orientation.w = est_q.w();
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.angular.z = 0.0;
    odom_pub_->publish(odom);

  }
  #pragma endregion EstimatedPositionBroadcast

  #pragma region SpeedControl

  auto forward_speed = cmd_vel_msg.linear.x;
  auto angular_speed = cmd_vel_msg.angular.z;

  auto command_motor_left =
      (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto command_motor_right =
      (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;

  wb_motor_set_velocity(left_motor, command_motor_left);
  wb_motor_set_velocity(right_motor, command_motor_right);

  #pragma endregion SpeedControl

}

} // namespace robot_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_controller::RobotController,
                       webots_ros2_driver::PluginInterface)