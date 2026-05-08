#ifndef CAR_CONTROLLER__CAR_CONTROLLER_NODE_HPP_
#define CAR_CONTROLLER__CAR_CONTROLLER_NODE_HPP_

#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

//  Dont forget to run the pigpiod demon
#include <pigpiod_if2.h>

namespace car_controller
{

const int PWM_MIN_US  = 1000;   
const int PWM_NEUTRAL = 1500;   
const int PWM_MAX_US  = 2000;  


class CarControllerNode : public rclcpp::Node
{
public:
  explicit CarControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CarControllerNode() override;

private:
  
  int    steering_pin_;
  int    throttle_pin_;
  int    pwm_min_us_;
  int    pwm_neutral_us_;
  int    pwm_max_us_;
  double max_linear_speed_;
  double max_angular_speed_;
  bool   steering_inverted_;
  bool   throttle_inverted_;
  double cmd_vel_timeout_;

  int  pi_handle_{-1};  
  bool hw_ok_{false};

  void initPigpio();
  void setPwm(unsigned int pin, int pulse_us);
  int  normalizedToUs(double value) const;
  static double clamp(double v, double lo, double hi);

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void watchdogCallback();

  // -------- ROS2 ----------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pwm_state_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Time last_cmd_time_;
};

}  // namespace car_controller

#endif  // CAR_CONTROLLER__CAR_CONTROLLER_NODE_HPP_