#include <Eigen/Dense>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/sensor_mag.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using namespace std::chrono_literals;

class EKF : public rclcpp::Node {
 public:
  EKF() : Node("ekf") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                           qos_profile);

    subscription_imu_ =
        this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined", qos,
            [this](px4_msgs::msg::SensorCombined::UniquePtr msg) {
              this->timestamp_ = msg->timestamp;
              this->gyr_x_ = msg->gyro_rad[0];
              this->gyr_y_ = msg->gyro_rad[1];
              this->gyr_z_ = msg->gyro_rad[2];
              this->acc_x_ = msg->accelerometer_m_s2[0];
              this->acc_y_ = msg->accelerometer_m_s2[1];
              this->acc_z_ = msg->accelerometer_m_s2[2];
            });

    subscription_mag_ = this->create_subscription<px4_msgs::msg::SensorMag>(
        "/fmu/out/sensor_mag", qos,
        [this](px4_msgs::msg::SensorMag::UniquePtr msg) {
          this->mag_x_ = msg->x;
          this->mag_y_ = msg->y;
          this->mag_z_ = msg->z;
        });

    subscription_attitude_ =
        this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            [this](px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
              this->q_w_ = msg->q[0];
              this->q_x_ = msg->q[1];
              this->q_y_ = msg->q[2];
              this->q_z_ = msg->q[3];
            });

    sensor_publisher_ =
        this->create_publisher<std_msgs::msg::String>("sensor_listener", 10);
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();
      message.data = std::string("\n") +
                     "gyr_x: " + std::to_string(this->gyr_x_) + " " +
                     "gyr_y: " + std::to_string(this->gyr_y_) + " " +
                     "gyr_z: " + std::to_string(this->gyr_z_) + "\n" +
                     "acc_x: " + std::to_string(this->acc_x_) + " " +
                     "acc_y: " + std::to_string(this->acc_y_) + " " +
                     "acc_z: " + std::to_string(this->acc_z_) + "\n" +
                     "mag_x: " + std::to_string(this->mag_x_) + " " +
                     "mag_y: " + std::to_string(this->mag_y_) + " " +
                     "mag_z: " + std::to_string(this->mag_z_) + "\n" +
                     "q_w: " + std::to_string(this->q_w_) + " " +
                     "q_x: " + std::to_string(this->q_x_) + " " +
                     "q_y: " + std::to_string(this->q_y_) + " " +
                     "q_z: " + std::to_string(this->q_z_) + "\n";
      RCLCPP_INFO(this->get_logger(), "'%s'", message.data.c_str());
      this->sensor_publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

 private:
  uint64_t timestamp_{};
  float gyr_x_{}, gyr_y_{}, gyr_z_{};
  float acc_x_{}, acc_y_{}, acc_z_{};
  float mag_x_{}, mag_y_{}, mag_z_{};
  float q_w_{}, q_x_{}, q_y_{}, q_z_{};

  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr
      subscription_imu_;
  rclcpp::Subscription<px4_msgs::msg::SensorMag>::SharedPtr subscription_mag_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr
      subscription_attitude_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_publisher_;
};

int main(int argc, char *argv[]) {
  std::cout << "Starting sensor listener node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKF>());
  rclcpp::shutdown();
  return 0;
}
