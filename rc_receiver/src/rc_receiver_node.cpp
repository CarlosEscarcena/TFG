#include "rc_receiver/rc_receiver_node.hpp"

#include <algorithm>
#include <cmath>

namespace rc_receiver
{

// ─────────────────────────────────────────────────────────────────────────────
// Free C function called by pigpio on its internal thread.
// Forwards the event to the node method using the user pointer.
// ─────────────────────────────────────────────────────────────────────────────
static void gpioCallbackDispatch(
  int /*pi*/, unsigned gpio, unsigned level, uint32_t tick, void * user)
{
  auto * node = static_cast<RcReceiverNode *>(user);
  node->onGpioChange(gpio, level, tick);
}


// Constructor
RcReceiverNode::RcReceiverNode(const rclcpp::NodeOptions & options)
: Node("rc_receiver_node", options)
{
  // ── Declare and load parameters ─────────
  this->declare_parameter<int>   ("throttle_pin", 13);
  this->declare_parameter<int>   ("steering_pin", 17);
  this->declare_parameter<int>   ("pwm_min_us", PWM_MIN_US);
  this->declare_parameter<int>   ("pwm_mid_us", PWM_MID_US);
  this->declare_parameter<int>   ("pwm_max_us", PWM_MAX_US);
  this->declare_parameter<int>   ("deadband_us", DEADBAND_US);
  this->declare_parameter<double>("max_linear_speed", 1.0);
  this->declare_parameter<double>("max_angular_speed", 1.0);
  this->declare_parameter<bool>  ("throttle_inverted", false);
  this->declare_parameter<bool>  ("steering_inverted", false);
  this->declare_parameter<double>("publish_rate", 50.0);   // Hz
  this->declare_parameter<double>("signal_timeout", 0.5);    // seconds

  throttle_pin_      = this->get_parameter("throttle_pin").as_int();
  steering_pin_      = this->get_parameter("steering_pin").as_int();
  pwm_min_us_        = this->get_parameter("pwm_min_us").as_int();
  pwm_mid_us_        = this->get_parameter("pwm_mid_us").as_int();
  pwm_max_us_        = this->get_parameter("pwm_max_us").as_int();
  deadband_us_       = this->get_parameter("deadband_us").as_int();
  max_linear_speed_  = this->get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
  throttle_inverted_ = this->get_parameter("throttle_inverted").as_bool();
  steering_inverted_ = this->get_parameter("steering_inverted").as_bool();
  publish_rate_      = this->get_parameter("publish_rate").as_double();
  signal_timeout_    = this->get_parameter("signal_timeout").as_double();

  last_throttle_time_ = this->now();
  last_steering_time_ = this->now();

 
  initPigpio();


  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10));

  raw_pwm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/rc_receiver/raw_pwm", rclcpp::QoS(10));

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&RcReceiverNode::publishTimerCallback, this)
  );

  char* pigpio_status;
  if (hw_ok_) {
    pigpio_status = "OK";
  } else {
    pigpio_status = "NOT WORKING";
  }

  RCLCPP_INFO(
    this->get_logger(),
    "RcReceiverNode started | throttle_pin=%d | steering_pin=%d | "
    "publish_rate=%.0f Hz | pigpio=%s",
    throttle_pin_, steering_pin_, publish_rate_, pigpio_status
  );
}


// Destructor
RcReceiverNode::~RcReceiverNode()
{
  if (hw_ok_ && pi_handle_ >= 0) {
    // Cancel GPIO callbacks before disconnecting
    if (throttle_cb_id_ >= 0) { callback_cancel(throttle_cb_id_); }
    if (steering_cb_id_  >= 0) { callback_cancel(steering_cb_id_);  }
    pigpio_stop(pi_handle_);
  }
  RCLCPP_INFO(this->get_logger(), "RcReceiverNode shut down.");
}


// initPigpio
void RcReceiverNode::initPigpio()
{
  pi_handle_ = pigpio_start(nullptr, nullptr);

  if (pi_handle_ < 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not connect to pigpiod (handle=%d)", pi_handle_);
    return;
  }

  // Configure pins
  set_mode(pi_handle_, static_cast<unsigned>(throttle_pin_), PI_INPUT);
  set_mode(pi_handle_, static_cast<unsigned>(steering_pin_),  PI_INPUT);
  set_pull_up_down(pi_handle_, static_cast<unsigned>(throttle_pin_), PI_PUD_DOWN);
  set_pull_up_down(pi_handle_, static_cast<unsigned>(steering_pin_),  PI_PUD_DOWN);

  // Register edge callbacks for pigpiod_if2
  throttle_cb_id_ = callback_ex(
    pi_handle_,
    static_cast<unsigned>(throttle_pin_),
    EITHER_EDGE, //(EITHER_EDGE = rising and falling)
    gpioCallbackDispatch,
    this
  );

  steering_cb_id_ = callback_ex(
    pi_handle_,
    static_cast<unsigned>(steering_pin_),
    EITHER_EDGE,
    gpioCallbackDispatch,
    this
  );

  if (throttle_cb_id_ < 0 || steering_cb_id_ < 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to register GPIO callbacks (throttle=%d, steering=%d).",
      throttle_cb_id_, steering_cb_id_);
    pigpio_stop(pi_handle_);
    pi_handle_ = -1;
    return;
  }

  hw_ok_ = true;
  RCLCPP_INFO(this->get_logger(), "pigpiod connected. Listening for PWM edges...");
}


// Measures pulse width: time between rising and falling edge
void RcReceiverNode::onGpioChange(unsigned gpio, unsigned level, uint32_t tick)
{
  // level=1 → rising edge  (start of pulse)
  // level=0 → falling edge (end of pulse)
  // level=2 → pigpio watchdog timeout (ignore)
  if (level == 2) { return; }

  const bool is_throttle = (static_cast<int>(gpio) == throttle_pin_);

  if (level == 1) {
    // Store the rising-edge tick
    if (is_throttle) {
      throttle_rise_tick_ = tick;
    } else {
      steering_rise_tick_ = tick;
    }
  } else {
    // Falling edge
    
    uint32_t rise_tick;
    if (is_throttle) {
      rise_tick = throttle_rise_tick_;
    } else {
      rise_tick = steering_rise_tick_;
    }

    int pulse_us = static_cast<int>(tick - rise_tick);  // difference in µs

    // Discard out-of-range pulses (noise or glitches)
    if (pulse_us < 500 || pulse_us > 2500) { return; }

    if (is_throttle) {
      throttle_pulse_us_.store(pulse_us);
      throttle_valid_.store(true);
      last_throttle_time_ = this->now();
    } else {
      steering_pulse_us_.store(pulse_us);
      steering_valid_.store(true);
      last_steering_time_ = this->now();
    }
  }
}


//publishes /cmd_vel at the configured rate
void RcReceiverNode::publishTimerCallback()
{
  const rclcpp::Time now = this->now();

  const double throttle_age = (now - last_throttle_time_).seconds();
  const double steering_age = (now - last_steering_time_).seconds();

  const bool throttle_ok = throttle_valid_.load() && (throttle_age < signal_timeout_);
  const bool steering_ok  = steering_valid_.load()  && (steering_age  < signal_timeout_);

  // The RC controlles is turned off
  if (!throttle_ok && !steering_ok) {
    static bool warned = false;
    if (!warned) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No RC signal. Check connections on GPIO %d and %d.",
        throttle_pin_, steering_pin_);
      warned = true;
    }
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  const int thr_us  = throttle_pulse_us_.load();
  const int str_us  = steering_pulse_us_.load();

  // Convert to normalised [-1, 1]
  double throttle_norm = pulseToNormalized(thr_us, pwm_min_us_, pwm_mid_us_, pwm_max_us_);
  double steering_norm  = pulseToNormalized(str_us,  pwm_min_us_, pwm_mid_us_, pwm_max_us_);

  // Apply deadband
  const double deadband_norm = static_cast<double>(deadband_us_) / (pwm_max_us_ - pwm_mid_us_);
  throttle_norm = applyDeadband(throttle_norm, deadband_norm);
  steering_norm  = applyDeadband(steering_norm,  deadband_norm);

  // Optional channel inversion
  if (throttle_inverted_) { throttle_norm = -throttle_norm; }
  if (steering_inverted_)  { steering_norm  = -steering_norm;  }

  // Scale to real-world speeds
  geometry_msgs::msg::Twist twist;
  twist.linear.x  = throttle_norm * max_linear_speed_;
  twist.angular.z = steering_norm  * max_angular_speed_;
  cmd_vel_pub_->publish(twist);

  // Publish raw PWM for diagnostics
  std_msgs::msg::Float32MultiArray raw;
  raw.data = {
    static_cast<float>(thr_us),
    static_cast<float>(str_us),
    static_cast<float>(throttle_norm),
    static_cast<float>(steering_norm)
  };
  raw_pwm_pub_->publish(raw);

  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "RC → thr=%dµs (%.2f)  str=%dµs (%.2f)",
    thr_us, throttle_norm, str_us, steering_norm);
}


// pulseToNormalized — maps a pulse width in µs to [-1, 1]
double RcReceiverNode::pulseToNormalized(
  int pulse_us, int min_us, int mid_us, int max_us) const
{
  double norm;
  if (pulse_us >= mid_us) {
    norm = static_cast<double>(pulse_us - mid_us) / (max_us - mid_us);
  } else {
    norm = static_cast<double>(pulse_us - mid_us) / (mid_us - min_us);
  }
  return clamp(norm, -1.0, 1.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// applyDeadband — forces output to 0 when the value is close to neutral
// ─────────────────────────────────────────────────────────────────────────────
double RcReceiverNode::applyDeadband(double value, double deadband_norm)
{
  // If the value is close enough to zero, treat it as zero
  if (std::abs(value) < deadband_norm) {
    return 0.0;
  }

  // Rescale so the output starts from 0 at the deadband edge
  double abs_value = std::abs(value);
  double rescaled  = (abs_value - deadband_norm) / (1.0 - deadband_norm);

  if (value > 0.0) {
    return rescaled;
  } else {
    return -rescaled;
  }
}

double RcReceiverNode::clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

}  // namespace rc_receiver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rc_receiver::RcReceiverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
