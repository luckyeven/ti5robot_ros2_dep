#pragma once

#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "can_hw.h"
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>

namespace ros2_control_t170 {

const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

// 类型定义
using scalar_t = double;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

inline std_msgs::msg::Float64MultiArray createFloat64MultiArrayFromVector(const vector_t& data) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(data.data(), data.data() + data.size());
  return msg;
}

struct Ti5MotorData {
  double pos_, vel_, tau_;                  // state
  double pos_des_, vel_des_, kp_, kd_, ff_; // command
};

struct Ti5ImuData {
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

class RobotSystem : public hardware_interface::SystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  bool initCan();
  void closeCanDevice(int dIndex);
  bool sendFixBodyCommand(int canIndex);
  bool setupJoints();
  bool setupImu();
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  hardware_interface::HardwareInfo info_;
  std::vector<Ti5MotorData> joint_data_;  // 23 个关节的状态
  Ti5ImuData imu_data_;
  std::vector<YKSMotorData> send_cmd_;    // 16 个电机的命令

  // 16 个电机的常数和方向
  const double motor_constants[16] = {51*0.118, 51*0.118, 51*0.175, 51*0.175, 51*0.118, 51*0.089,
                                      51*0.118, 51*0.118, 51*0.175, 51*0.175, 51*0.118, 51*0.089,
                                      101*0.096, 101*0.089, 101*0.096, 101*0.089};
  const std::vector<int> direction_motor_{-1, 1, -1, -1, -1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  float leg3_kp_scale, leg3_kd_scale, leg4_kp_scale, leg4_kd_scale, leg5_kp_scale, leg5_kd_scale;

  bool use_imu_topic_ = true;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Node::SharedPtr node_;

  vector_t motor_pos_feedback_;
  vector_t motor_vel_feedback_;
  vector_t motor_tau_feedback_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_torque_pub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  float bias_motor_[12] = {0, 0, 0.523599, -1.047197, 0.523599, 0, 0, 0.523599, -1.047197, 0.523599};
};

}  // namespace ros2_control_t170
