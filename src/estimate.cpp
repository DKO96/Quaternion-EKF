#include <math.h>
#include <stdint.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "ekf_imu/ekf.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class EstimateQuaternion : public rclcpp::Node {
 public:
  EstimateQuaternion() : Node("estimate") {
    subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/sensor/imu_data", 10, [this](sensor_msgs::msg::Imu::UniquePtr msg) {
          // this->timestamp_ = msg->timestamp;
          this->gyr_ << msg->angular_velocity.x, msg->angular_velocity.y,
              msg->angular_velocity.z;
          this->acc_ << msg->linear_acceleration.x, msg->linear_acceleration.y,
              msg->linear_acceleration.z;
        });

    subscription_mag_ =
        this->create_subscription<sensor_msgs::msg::MagneticField>(
            "/sensor/mag_data", 10,
            [this](sensor_msgs::msg::MagneticField::UniquePtr msg) {
              this->mag_ << msg->magnetic_field.x, msg->magnetic_field.y,
                  msg->magnetic_field.z;
            });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ =
        this->create_wall_timer(10ms, [this]() { this->estimate_callback(); });
  }

 private:
  Eigen::Vector3d gyr_, acc_, mag_;
  bool init_ekf = false;

  EKF ekf;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr
      subscription_mag_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void estimate_callback() {
    // gyr_ = {0.00, 0.03, -0.01};
    // acc_ = {-0.39, 0.28, 9.69};
    // mag_ = {4.2, -35.7, -62.25};

    if (!init_ekf) {
      ekf.initial_state(acc_, mag_);
      init_ekf = true;
      return;
    }

    Eigen::Vector4d q = ekf.update(gyr_, acc_, mag_, 0.01);
    std::cout << "q: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
              << std::endl;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "sensor_link";

    t.transform.rotation.w = q(0);
    t.transform.rotation.x = q(0);
    t.transform.rotation.y = q(0);
    t.transform.rotation.z = q(0);

    tf_broadcaster_->sendTransform(t);
  }
};

int main(int argc, char *argv[]) {
  std::cout << "Starting quaternion tf node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstimateQuaternion>());

  rclcpp::shutdown();
  return 0;
}