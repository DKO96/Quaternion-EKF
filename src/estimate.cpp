#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
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

class EstimateQuaternionNode : public rclcpp::Node {
 public:
  EstimateQuaternionNode() : Node("estimate") {
    rclcpp::QoS qos = rclcpp::QoS(10);

    imu_sub_.subscribe(this, "/sensor/imu", qos.get_rmw_qos_profile());
    mag_sub_.subscribe(this, "/sensor/mag", qos.get_rmw_qos_profile());

    uint32_t queue_size = 10;
    sync_ = std::make_shared<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField>>(
        imu_sub_, mag_sub_, queue_size);

    sync_->registerCallback(
        std::bind(&EstimateQuaternionNode::estimate_quaternion, this,
                  std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

 private:
  Eigen::Vector3d gyr_, acc_, mag_;
  bool init_ekf = false;
  rclcpp::Time last_stamp;
  rclcpp::Time stamp;

  EKF ekf;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
  message_filters::Subscriber<sensor_msgs::msg::MagneticField> mag_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<
      sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField>>
      sync_;

  void estimate_quaternion(
      const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
      const sensor_msgs::msg::MagneticField::ConstSharedPtr &mag_msg) {
    stamp = imu_msg->header.stamp;
    gyr_ << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z;
    acc_ << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z;
    mag_ << mag_msg->magnetic_field.x, mag_msg->magnetic_field.y,
        mag_msg->magnetic_field.z;

    if (!init_ekf) {
      ekf.initial_state(acc_, mag_);
      init_ekf = true;
      last_stamp = stamp;
      return;
    }

    double dt = last_stamp == rclcpp::Time(0, 0, RCL_ROS_TIME)
                    ? 0.01
                    : (stamp - last_stamp).seconds();
    last_stamp = stamp;
    dt = dt <= 0.0 ? 0.01 : dt;

    Eigen::Vector4d q = ekf.update(gyr_, acc_, mag_, dt);
    std::cout << "q: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
              << std::endl;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "sensor_link";

    t.transform.rotation.w = q(0);
    t.transform.rotation.x = q(1);
    t.transform.rotation.y = q(2);
    t.transform.rotation.z = q(3);

    tf_broadcaster_->sendTransform(t);
  }
};

int main(int argc, char *argv[]) {
  std::cout << "Starting quaternion tf node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstimateQuaternionNode>());

  rclcpp::shutdown();
  return 0;
}