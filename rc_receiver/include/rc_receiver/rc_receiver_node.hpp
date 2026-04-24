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

// ─────────────────────────────────────────────────────────────────────────────
// Constantes PWM estándar hobby (µs)
// ─────────────────────────────────────────────────────────────────────────────
constexpr int PWM_MIN_US  = 1000;
constexpr int PWM_MID_US  = 1500;
constexpr int PWM_MAX_US  = 2000;

// Zona muerta alrededor del neutro (µs): evita ruido cerca de 1500
constexpr int DEADBAND_US = 30;

// ─────────────────────────────────────────────────────────────────────────────
/// @brief Nodo que lee dos canales PWM del receptor RC via pigpio callbacks
///        y publica geometry_msgs/Twist en /cmd_vel
// ─────────────────────────────────────────────────────────────────────────────
class RcReceiverNode : public rclcpp::Node
{
public:
  explicit RcReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RcReceiverNode() override;

  // Llamado por pigpio desde su hilo interno → debe ser público
  void onGpioChange(unsigned gpio, unsigned level, uint32_t tick);

private:
  void initPigpio();
  void publishTimerCallback();

  // Convierte pulso PWM [min,max] → valor normalizado [-1, 1]
  double pulseToNormalized(int pulse_us, int min_us, int mid_us, int max_us) const;

  static double clamp(double v, double lo, double hi);
  static double applyDeadband(double value, double deadband_norm);

  // ── Parámetros ─────────────────────────────────────────────────────────────
  int    throttle_pin_;       // GPIO 13
  int    steering_pin_;       // GPIO 17
  int    pwm_min_us_;
  int    pwm_mid_us_;
  int    pwm_max_us_;
  int    deadband_us_;
  double max_linear_speed_;
  double max_angular_speed_;
  bool   throttle_inverted_;
  bool   steering_inverted_;
  double publish_rate_;
  double signal_timeout_;     // segundos sin señal → publica cero

  // ── Estado de los canales (escritos por callback pigpio, leídos por timer) ─
  // Usamos atomic para acceso seguro entre hilos
  std::atomic<int>      throttle_pulse_us_{PWM_MID_US};
  std::atomic<int>      steering_pulse_us_{PWM_MID_US};
  std::atomic<bool>     throttle_valid_{false};
  std::atomic<bool>     steering_valid_{false};

  // Timestamps del último flanco (para medir ancho de pulso)
  uint32_t throttle_rise_tick_{0};
  uint32_t steering_rise_tick_{0};

  // Timestamp ROS del último pulso válido recibido
  rclcpp::Time last_throttle_time_;
  rclcpp::Time last_steering_time_;

  // ── pigpio ─────────────────────────────────────────────────────────────────
  int  pi_handle_{-1};
  bool hw_ok_{false};

  // Callbacks registrados en pigpio (necesitamos guardar el ID para cancelarlos)
  int throttle_cb_id_{-1};
  int steering_cb_id_{-1};

  // ── ROS2 ───────────────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr raw_pwm_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace rc_receiver

#endif  // RC_RECEIVER__RC_RECEIVER_NODE_HPP_