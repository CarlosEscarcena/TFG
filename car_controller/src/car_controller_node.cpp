#include "car_controller/car_controller_node.hpp"

#include <algorithm>
#include <array>

namespace car_controller
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
CarControllerNode::CarControllerNode(const rclcpp::NodeOptions & options)
: Node("car_controller_node", options)
{
  // ── Declarar y leer parámetros ──────────────────────────────────────────
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

  // ── Iniciar pigpio ──────────────────────────────────────────────────────
  initPigpio();

  // ── Suscriptor /cmd_vel ─────────────────────────────────────────────────
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(10),
    std::bind(&CarControllerNode::cmdVelCallback, this, std::placeholders::_1)
  );

  // ── Publicador de estado PWM ────────────────────────────────────────────
  pwm_state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/car_controller/pwm_state",
    rclcpp::QoS(10)
  );

  // ── Watchdog (100 ms) ───────────────────────────────────────────────────
  last_cmd_time_ = this->now();
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CarControllerNode::watchdogCallback, this)
  );

  // ── Posición inicial: neutro ────────────────────────────────────────────
  setPwm(static_cast<unsigned>(steering_pin_), pwm_neutral_us_);
  setPwm(static_cast<unsigned>(throttle_pin_), pwm_neutral_us_);

  RCLCPP_INFO(
    this->get_logger(),
    "CarControllerNode iniciado | steering_pin=%d | throttle_pin=%d | pigpio=%s",
    steering_pin_, throttle_pin_, hw_ok_ ? "OK" : "SIMULADO"
  );
}

// ─────────────────────────────────────────────────────────────────────────────
// Destructor — siempre dejar el coche parado
// ─────────────────────────────────────────────────────────────────────────────
CarControllerNode::~CarControllerNode()
{
  RCLCPP_INFO(this->get_logger(), "Apagando CarControllerNode...");

  setPwm(static_cast<unsigned>(steering_pin_), pwm_neutral_us_);
  setPwm(static_cast<unsigned>(throttle_pin_), pwm_neutral_us_);

  if (hw_ok_ && pi_handle_ >= 0) {
    // Apagar PWM antes de desconectar
    set_servo_pulsewidth(pi_handle_, static_cast<unsigned>(steering_pin_), 0);
    set_servo_pulsewidth(pi_handle_, static_cast<unsigned>(throttle_pin_), 0);
    pigpio_stop(pi_handle_);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// initPigpio
// ─────────────────────────────────────────────────────────────────────────────
void CarControllerNode::initPigpio()
{
  // pigpio_start(host, port) → conecta a pigpiod
  // nullptr, nullptr → localhost:8888 (por defecto)
  pi_handle_ = pigpio_start(nullptr, nullptr);

  if (pi_handle_ < 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "No se pudo conectar a pigpiod (handle=%d). "
      "Asegúrate de que 'pigpiod' está corriendo en el host. "
      "Modo SIMULACIÓN activado.",
      pi_handle_
    );
    return;
  }

  // Configurar pines como salidas
  set_mode(pi_handle_, static_cast<unsigned>(steering_pin_), PI_OUTPUT);
  set_mode(pi_handle_, static_cast<unsigned>(throttle_pin_), PI_OUTPUT);

  hw_ok_ = true;
  RCLCPP_INFO(this->get_logger(), "pigpiod conectado correctamente (handle=%d).", pi_handle_);
}

// ─────────────────────────────────────────────────────────────────────────────
// setPwm — envía pulso PWM en µs al pin
// ─────────────────────────────────────────────────────────────────────────────
void CarControllerNode::setPwm(unsigned int pin, int pulse_us)
{
  const int clamped = static_cast<int>(
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
  } else {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[SIM] pin=%u  pulso=%d µs", pin, clamped
    );
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// normalizedToUs — [-1,1] → [pwm_min, pwm_max] µs
// ─────────────────────────────────────────────────────────────────────────────
int CarControllerNode::normalizedToUs(double value) const
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

// ─────────────────────────────────────────────────────────────────────────────
// clamp — utilidad inline
// ─────────────────────────────────────────────────────────────────────────────
double CarControllerNode::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

// ─────────────────────────────────────────────────────────────────────────────
// cmdVelCallback — suscriptor /cmd_vel
// ─────────────────────────────────────────────────────────────────────────────
void CarControllerNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_time_ = this->now();

  // Normalizar
  double throttle_norm = clamp(msg->linear.x  / max_linear_speed_,  -1.0, 1.0);
  double steering_norm = clamp(msg->angular.z / max_angular_speed_, -1.0, 1.0);

  // Inversión opcional de canales
  if (throttle_inverted_) { throttle_norm = -throttle_norm; }
  if (steering_inverted_) { steering_norm = -steering_norm; }

  // Convertir a µs
  const int throttle_us = normalizedToUs(throttle_norm);
  const int steering_us = normalizedToUs(steering_norm);

  // Enviar PWM
  setPwm(static_cast<unsigned>(throttle_pin_), throttle_us);
  setPwm(static_cast<unsigned>(steering_pin_), steering_us);

  // Publicar estado (diagnóstico)
  std_msgs::msg::Float32MultiArray state_msg;
  state_msg.data = {
    static_cast<float>(steering_us),
    static_cast<float>(throttle_us),
    static_cast<float>(steering_norm),
    static_cast<float>(throttle_norm)
  };
  pwm_state_pub_->publish(state_msg);

  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 200,
    "CMD → throttle=%dµs (%.2f)  steering=%dµs (%.2f)",
    throttle_us, throttle_norm, steering_us, steering_norm
  );
}

// ─────────────────────────────────────────────────────────────────────────────
// watchdogCallback — para el coche si no llegan comandos
// ─────────────────────────────────────────────────────────────────────────────
void CarControllerNode::watchdogCallback()
{
  const double elapsed =
    (this->now() - last_cmd_time_).seconds();

  if (elapsed > cmd_vel_timeout_) {
    setPwm(static_cast<unsigned>(steering_pin_), pwm_neutral_us_);
    setPwm(static_cast<unsigned>(throttle_pin_), pwm_neutral_us_);
  }
}

}  // namespace car_controller

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_controller::CarControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}