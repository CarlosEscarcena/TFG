#ifndef CAR_CONTROLLER__CAR_CONTROLLER_NODE_HPP_
#define CAR_CONTROLLER__CAR_CONTROLLER_NODE_HPP_

#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// pigpiod_if2: interfaz de cliente al daemon pigpiod (recomendada en Docker)
// No requiere privilegios root directamente; se conecta por socket TCP/Unix.
#include <pigpiod_if2.h>

namespace car_controller
{

// ─────────────────────────────────────────────────────────────────────────────
// Constantes PWM (microsegundos)
// ─────────────────────────────────────────────────────────────────────────────
constexpr int PWM_MIN_US  = 1000;   ///< Pulso mínimo  → reversa / giro máx. izq.
constexpr int PWM_NEUTRAL = 1500;   ///< Pulso neutro  → parado / recto
constexpr int PWM_MAX_US  = 2000;   ///< Pulso máximo  → avance / giro máx. der.

// ─────────────────────────────────────────────────────────────────────────────
/// @brief Nodo ROS2 que convierte geometry_msgs/Twist → señales PWM servo/ESC
// ─────────────────────────────────────────────────────────────────────────────
class CarControllerNode : public rclcpp::Node
{
public:
  explicit CarControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CarControllerNode() override;

private:
  // ── Métodos internos ───────────────────────────────────────────────────────
  void initPigpio();
  void setPwm(unsigned int pin, int pulse_us);
  int  normalizedToUs(double value) const;
  static double clamp(double v, double lo, double hi);

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void watchdogCallback();

  // ── Parámetros ─────────────────────────────────────────────────────────────
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

  // ── Estado pigpio ──────────────────────────────────────────────────────────
  int  pi_handle_{-1};   ///< Handle de conexión a pigpiod (-1 = no conectado)
  bool hw_ok_{false};

  // ── ROS2 ───────────────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pwm_state_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Time last_cmd_time_;
};

}  // namespace car_controller

#endif  // CAR_CONTROLLER__CAR_CONTROLLER_NODE_HPP_