#include <math.h>
#include <stdint.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class QuaternionTF : public rclcpp::Node {
 public:
  QuaternionTF() : Node("quaternion_tf") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                           qos_profile);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto quaternion_callback_ =
        [this](const std::shared_ptr<px4_msgs::msg::VehicleAttitude> msg) {
          geometry_msgs::msg::TransformStamped t;

          t.header.stamp = this->get_clock()->now();
          t.header.frame_id = "map";
          t.child_frame_id = "px4_quaternion";

          t.transform.translation.x = 0.0;
          t.transform.translation.y = 0.0;
          t.transform.translation.z = 0.0;

          this->q_ned_frd.w() = msg->q[0];
          this->q_ned_frd.x() = msg->q[1];
          this->q_ned_frd.y() = msg->q[2];
          this->q_ned_frd.z() = msg->q[3];

          this->q_enu_flu = this->q_enu_ned * this->q_ned_frd * this->q_frd_flu;

          t.transform.rotation.x = this->q_enu_flu.x();
          t.transform.rotation.y = this->q_enu_flu.y();
          t.transform.rotation.z = this->q_enu_flu.z();
          t.transform.rotation.w = this->q_enu_flu.w();
          tf_broadcaster_->sendTransform(t);
        };

    attitude_subscriber_ =
        this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos, quaternion_callback_);
  }

 private:
  Eigen::Quaterniond q_ned_frd;
  Eigen::Quaterniond q_enu_flu;
  const Eigen::Quaterniond q_enu_ned{0, M_SQRT1_2, M_SQRT1_2, 0};
  const Eigen::Quaterniond q_frd_flu{0, 1, 0, 0};

  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr
      attitude_subscriber_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  std::cout << "Starting quaternion tf node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuaternionTF>());

  rclcpp::shutdown();
  return 0;
}