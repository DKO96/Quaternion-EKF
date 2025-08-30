#include <math.h>
#include <stdint.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/sensor_mag.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ekf_imu/ekf.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class EstimateQuaternion : public rclcpp::Node {
 public:
  EstimateQuaternion() : Node("estimate") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                           qos_profile);

    subscription_imu_ =
        this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined", qos,
            [this](px4_msgs::msg::SensorCombined::UniquePtr msg) {
              // this->timestamp_ = msg->timestamp;
              this->gyr_ << msg->gyro_rad[0], msg->gyro_rad[1],
                  msg->gyro_rad[2];
              this->acc_ << msg->accelerometer_m_s2[0],
                  msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2];
            });

    subscription_mag_ = this->create_subscription<px4_msgs::msg::SensorMag>(
        "/fmu/out/sensor_mag", qos,
        [this](px4_msgs::msg::SensorMag::UniquePtr msg) {
          this->mag_ << msg->x, msg->y, msg->z;
        });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // auto px4_quaternion_callback_ =
    //     [this](const std::shared_ptr<px4_msgs::msg::VehicleAttitude> msg) {
    //       geometry_msgs::msg::TransformStamped t;

    //       t.header.stamp = this->get_clock()->now();
    //       t.header.frame_id = "map";
    //       t.child_frame_id = "px4_quaternion";

    //       t.transform.translation.x = 0.0;
    //       t.transform.translation.y = 0.0;
    //       t.transform.translation.z = 0.0;

    //       this->q_ned_frd.w() = msg->q[0];
    //       this->q_ned_frd.x() = msg->q[1];
    //       this->q_ned_frd.y() = msg->q[2];
    //       this->q_ned_frd.z() = msg->q[3];

    //       this->q_enu_flu = this->q_enu_ned * this->q_ned_frd *
    //       this->q_frd_flu;

    //       t.transform.rotation.x = this->q_enu_flu.x();
    //       t.transform.rotation.y = this->q_enu_flu.y();
    //       t.transform.rotation.z = this->q_enu_flu.z();
    //       t.transform.rotation.w = this->q_enu_flu.w();
    //       tf_broadcaster_->sendTransform(t);
    //     };

    // attitude_subscriber_ =
    //     this->create_subscription<px4_msgs::msg::VehicleAttitude>(
    //         "/fmu/out/vehicle_attitude", qos, px4_quaternion_callback_);

    timer_ =
        this->create_wall_timer(10ms, [this]() { this->estimate_callback(); });
  }

 private:
  Eigen::Quaterniond q_ned_frd;
  Eigen::Quaterniond q_enu_flu;
  const Eigen::Quaterniond q_enu_ned{0, M_SQRT1_2, M_SQRT1_2, 0};
  const Eigen::Quaterniond q_frd_flu{0, 1, 0, 0};

  Eigen::Vector3d gyr_, acc_, mag_;
  bool init_ekf = false;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr
      subscription_imu_;
  rclcpp::Subscription<px4_msgs::msg::SensorMag>::SharedPtr subscription_mag_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr
      attitude_subscriber_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  EKF ekf;

  void estimate_callback() {
    if (!init_ekf) {
      Eigen::Vector4d q_init = ekf.initial_state(acc_, mag_);
      q_ned_frd =
          Eigen::Quaterniond(q_init(0), q_init(1), q_init(2), q_init(3));
      init_ekf = true;
      return;
    }

    Eigen::Vector4d q_est = ekf.update(gyr_, acc_, mag_, 0.01);
    q_ned_frd = Eigen::Quaterniond(q_est(0), q_est(1), q_est(2), q_est(3));

    q_enu_flu = q_enu_ned * q_ned_frd * q_frd_flu;

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "est_quaternion";

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = q_enu_flu.x();
    t.transform.rotation.y = q_enu_flu.y();
    t.transform.rotation.z = q_enu_flu.z();
    t.transform.rotation.w = q_enu_flu.w();
    tf_broadcaster_->sendTransform(t);

    std::cout << "q: " << q_enu_flu.w() << ", " << q_enu_flu.x() << ", "
              << q_enu_flu.y() << ", " << q_enu_flu.z() << std::endl;
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