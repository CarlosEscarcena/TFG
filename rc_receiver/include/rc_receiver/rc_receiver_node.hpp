#ifndef RC_RECEIVER__RC_RECEIVER_NODE_HPP_
#define RC_RECEIVER__RC_RECEIVER_NODE_HPP_

#include <atomic>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <pigpiod_if2.h>

namespace rc_receiver
{

constexpr int PWM_MIN_US  = 1000;   
constexpr int PWM_MID_US  = 1500;   
constexpr int PWM_MAX_US  = 2000;   

constexpr int DEADBAND_US = 30;     


class RcReceiverNode : public rclcpp::Node
{
public:

  explicit RcReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RcReceiverNode() override;

  void onGpioChange(unsigned gpio, unsigned level, uint32_t tick);

private:

  // -- Parameters -------
  int    throttle_pin_;        
  int    steering_pin_;        
  int    pwm_min_us_;          
  int    pwm_mid_us_;          
  int    pwm_max_us_;          
  int    deadband_us_;         
  double max_linear_speed_;    
  double max_angular_speed_;  
  bool   throttle_inverted_;   
  bool   steering_inverted_;   
  double publish_rate_;        
  double signal_timeout_;      

  // -- Channel state --------
  // Atomics ensure safe cross-thread access without a mutex
  std::atomic<int>      throttle_pulse_us_{PWM_MID_US};
  std::atomic<int>      steering_pulse_us_{PWM_MID_US};
  std::atomic<bool>     throttle_valid_{false};
  std::atomic<bool>     steering_valid_{false};

  //used to measure pulse width
  uint32_t throttle_rise_tick_{0};
  uint32_t steering_rise_tick_{0};

  // -- pigpio ------
  int  pi_handle_{-1};   
  bool hw_ok_{false};  

  // Callback IDs returned by pigpio 
  int throttle_cb_id_{-1};
  int steering_cb_id_{-1};

  // -- Funcions ------
  void initPigpio();
  void publishTimerCallback();

  double pulseToNormalized(int pulse_us, int min_us, int mid_us, int max_us) const;

  static double clamp(double v, double lo, double hi);
  static double applyDeadband(double value, double deadband_norm);

  // -- ROS2 -------
  //timestamp of the last valid pulse received per channel
  rclcpp::Time last_throttle_time_;
  rclcpp::Time last_steering_time_;

  // Publishers and timer
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr raw_pwm_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace rc_receiver

#endif  // RC_RECEIVER__RC_RECEIVER_NODE_HPP_
