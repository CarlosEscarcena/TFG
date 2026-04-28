#include "car_controller/car_controller_node.hpp"

#include <algorithm>
#include <array>

namespace car_controller
{


CarControllerNode::CarControllerNode(const rclcpp::NodeOptions & options)
: Node("car_controller_node", options)
{

  // ───────────────── Read all the parameters ───────────────────
  this->declare_parameter<int>   ("steering_pin",      12);
  this->declare_parameter<int>   ("throttle_pin",      19);
  this->declare_parameter<int>   ("pwm_min_us",        PWM_MIN_US);
  this->declare_parameter<int>   ("pwm_neutral_us",    PWM_NEUTRAL);
  this->declare_parameter<int>   ("pwm_max_us",        PWM_MAX_US);
  this->declare_parameter<double>("max_linear_speed",  1.0);
  this->declare_parameter<double>("max_angular_speed", 1.0);
  this->declare_parameter<bool>  ("steering_inverted", false);
  this->declare_parameter<bool>  ("throttle_inverted", false);
  this->declare_parameter<double>("cmd_vel_timeout",   0.5);

  steering_pin_      = this->get_parameter("steering_pin").as_int();
  throttle_pin_      = this->get_parameter("throttle_pin").as_int();
  pwm_min_us_        = this->get_parameter("pwm_min_us").as_int();
  pwm_neutral_us_    = this->get_parameter("pwm_neutral_us").as_int();
  pwm_max_us_        = this->get_parameter("pwm_max_us").as_int();
  max_linear_speed_  = this->get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
  steering_inverted_ = this->get_parameter("steering_inverted").as_bool();
  throttle_inverted_ = this->get_parameter("throttle_inverted").as_bool();
  cmd_vel_timeout_   = this->get_parameter("cmd_vel_timeout").as_double();


  initPigpio();

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(10),
    std::bind(&CarControllerNode::cmdVelCallback, this, std::placeholders::_1)
  );

  pwm_state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/car_controller/pwm_state",
    rclcpp::QoS(10)
  );

  // Watchdog
  last_cmd_time_ = this->now();
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CarControllerNode::watchdogCallback, this)
  );

  // Set to initial pose
  setPwm(static_cast<unsigned>(steering_pin_), pwm_neutral_us_);
  setPwm(static_cast<unsigned>(throttle_pin_), pwm_neutral_us_);

  const char* pigpio_status;
  if (hw_ok_) {
    pigpio_status = "OK";
  } else {
    pigpio_status = "NOT WORKING";
  }

  RCLCPP_INFO(
    this->get_logger(),
    "CarControllerNode running | steering_pin=%d | throttle_pin=%d | pigpio=%s",
    steering_pin_, throttle_pin_, pigpio_status
  );

}


// Destructor 
CarControllerNode::~CarControllerNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down CarControllerNode...");

  // Set to neutral pose
  setPwm(static_cast<unsigned>(steering_pin_), pwm_neutral_us_);
  setPwm(static_cast<unsigned>(throttle_pin_), pwm_neutral_us_);

  if (hw_ok_ && pi_handle_ >= 0) {
    // Disable PWM before disconnecting
    set_servo_pulsewidth(pi_handle_, static_cast<unsigned>(steering_pin_), 0);
    set_servo_pulsewidth(pi_handle_, static_cast<unsigned>(throttle_pin_), 0);
    pigpio_stop(pi_handle_);
  }
}


//InitPigpio
void CarControllerNode::initPigpio()
{
  pi_handle_ = pigpio_start(nullptr, nullptr);

  if (pi_handle_ < 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not connect to pigpiod (handle=%d). "
      "Make sure 'pigpiod' is running on the host. ",
      pi_handle_
    );
    return;
  }

  // Configure pins as outputs
  set_mode(pi_handle_, static_cast<unsigned>(steering_pin_), PI_OUTPUT);
  set_mode(pi_handle_, static_cast<unsigned>(throttle_pin_), PI_OUTPUT);

  hw_ok_ = true;
  RCLCPP_INFO(this->get_logger(), "pigpiod connected successfully (handle=%d).", pi_handle_);
}


//Sends a PWM pulse in µs to the given pin
void CarControllerNode::setPwm(unsigned int pin, int pulse_us)
{
  int clamped = static_cast<int>(
    clamp(static_cast<double>(pulse_us),
          static_cast<double>(pwm_min_us_),
          static_cast<double>(pwm_max_us_))
  );

  if (hw_ok_ && pi_handle_ >= 0) {

    int ret = set_servo_pulsewidth(pi_handle_, pin, static_cast<unsigned>(clamped));

    if (ret < 0) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Error set_servo_pulsewidth(pin=%u, pulse=%d): %d", pin, clamped, ret
      );
    }
  } 
}


// normalizedToµs — [-1,1] → [pwm_min, pwm_max] µs
int CarControllerNode::normalizedToUs(double value)
{
  value = clamp(value, -1.0, 1.0);
  double us;
  if (value >= 0.0) {
    us = pwm_neutral_us_ + value * (pwm_max_us_ - pwm_neutral_us_);
  } else {
    us = pwm_neutral_us_ + value * (pwm_neutral_us_ - pwm_min_us_);
  }
  return static_cast<int>(std::round(us));
}


double CarControllerNode::clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}


// cmdVelCallback 
void CarControllerNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_time_ = this->now();

  // Normalise
  double throttle_norm = clamp(msg->linear.x  / max_linear_speed_,  -1.0, 1.0);
  double steering_norm = clamp(msg->angular.z / max_angular_speed_, -1.0, 1.0);

  // Inversion if needed
  if (throttle_inverted_) { 
    throttle_norm = -throttle_norm; 
  }
  if (steering_inverted_) { 
    steering_norm = -steering_norm; 
  }

  // Convert to µs
  const int throttle_us = normalizedToUs(throttle_norm);
  const int steering_us = normalizedToUs(steering_norm);

  // Send PWM
  setPwm(static_cast<unsigned>(throttle_pin_), throttle_us);
  setPwm(static_cast<unsigned>(steering_pin_), steering_us);

  // Publish state 
  std_msgs::msg::Float32MultiArray state_msg;
  state_msg.data = {
    static_cast<float>(steering_us),
    static_cast<float>(throttle_us),
    static_cast<float>(steering_norm),
    static_cast<float>(throttle_norm)
  };
  pwm_state_pub_->publish(state_msg);

}


// watchdogCallback — stops the car if no commands are recieved
void CarControllerNode::watchdogCallback()
{
  double elapsed =
    (this->now() - last_cmd_time_).seconds();

  if (elapsed > cmd_vel_timeout_) {
    //Stop the car
    setPwm(static_cast<unsigned>(steering_pin_), pwm_neutral_us_);
    setPwm(static_cast<unsigned>(throttle_pin_), pwm_neutral_us_);
  }
}

}  // namespace car_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_controller::CarControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
