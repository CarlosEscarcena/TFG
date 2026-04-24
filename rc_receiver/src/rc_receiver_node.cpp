#include "rc_receiver/rc_receiver_node.hpp"

#include <algorithm>
#include <cmath>

namespace rc_receiver
{

// ─────────────────────────────────────────────────────────────────────────────
// Función C libre que pigpio llama en su hilo interno.
// Redirige al método del nodo usando el puntero de usuario.
// ─────────────────────────────────────────────────────────────────────────────
static void gpioCallbackDispatch(
  int /*pi*/, unsigned gpio, unsigned level, uint32_t tick, void * user)
{
  auto * node = static_cast<RcReceiverNode *>(user);
  node->onGpioChange(gpio, level, tick);
}

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
RcReceiverNode::RcReceiverNode(const rclcpp::NodeOptions & options)
: Node("rc_receiver_node", options)
{
  // ── Parámetros ──────────────────────────────────────────────────────────
  this->declare_parameter<int>   ("throttle_pin",      13);
  this->declare_parameter<int>   ("steering_pin",      17);
  this->declare_parameter<int>   ("pwm_min_us",        PWM_MIN_US);
  this->declare_parameter<int>   ("pwm_mid_us",        PWM_MID_US);
  this->declare_parameter<int>   ("pwm_max_us",        PWM_MAX_US);
  this->declare_parameter<int>   ("deadband_us",       DEADBAND_US);
  this->declare_parameter<double>("max_linear_speed",  1.0);
  this->declare_parameter<double>("max_angular_speed", 1.0);
  this->declare_parameter<bool>  ("throttle_inverted", false);
  this->declare_parameter<bool>  ("steering_inverted", false);
  this->declare_parameter<double>("publish_rate",      50.0);   // Hz
  this->declare_parameter<double>("signal_timeout",    0.5);    // seg

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

  // ── pigpio ──────────────────────────────────────────────────────────────
  initPigpio();

  // ── Publishers ──────────────────────────────────────────────────────────
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10));

  raw_pwm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/rc_receiver/raw_pwm", rclcpp::QoS(10));

  // ── Timer de publicación ─────────────────────────────────────────────────
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&RcReceiverNode::publishTimerCallback, this)
  );

  RCLCPP_INFO(
    this->get_logger(),
    "RcReceiverNode iniciado | throttle_pin=%d | steering_pin=%d | "
    "publish_rate=%.0f Hz | pigpio=%s",
    throttle_pin_, steering_pin_, publish_rate_,
    hw_ok_ ? "OK" : "SIMULADO"
  );
}

// ─────────────────────────────────────────────────────────────────────────────
// Destructor
// ─────────────────────────────────────────────────────────────────────────────
RcReceiverNode::~RcReceiverNode()
{
  if (hw_ok_ && pi_handle_ >= 0) {
    // Cancelar callbacks de GPIO
    if (throttle_cb_id_ >= 0) { callback_cancel(throttle_cb_id_); }
    if (steering_cb_id_  >= 0) { callback_cancel(steering_cb_id_);  }
    pigpio_stop(pi_handle_);
  }
  RCLCPP_INFO(this->get_logger(), "RcReceiverNode apagado.");
}

// ─────────────────────────────────────────────────────────────────────────────
// initPigpio
// ─────────────────────────────────────────────────────────────────────────────
void RcReceiverNode::initPigpio()
{
  pi_handle_ = pigpio_start(nullptr, nullptr);

  if (pi_handle_ < 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "No se pudo conectar a pigpiod (handle=%d). Modo SIMULACIÓN.", pi_handle_);
    return;
  }

  // Configurar pines como entradas con pull-down
  set_mode(pi_handle_, static_cast<unsigned>(throttle_pin_), PI_INPUT);
  set_mode(pi_handle_, static_cast<unsigned>(steering_pin_),  PI_INPUT);
  set_pull_up_down(pi_handle_, static_cast<unsigned>(throttle_pin_), PI_PUD_DOWN);
  set_pull_up_down(pi_handle_, static_cast<unsigned>(steering_pin_),  PI_PUD_DOWN);

  // Registrar callbacks para detectar flancos (EITHER = subida Y bajada)
  // pigpiod_if2 usa callback_ex con puntero de usuario
  throttle_cb_id_ = callback_ex(
    pi_handle_,
    static_cast<unsigned>(throttle_pin_),
    EITHER_EDGE,
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
      "Error registrando callbacks GPIO (throttle=%d, steering=%d).",
      throttle_cb_id_, steering_cb_id_);
    pigpio_stop(pi_handle_);
    pi_handle_ = -1;
    return;
  }

  hw_ok_ = true;
  RCLCPP_INFO(this->get_logger(), "pigpiod conectado. Escuchando flancos PWM...");
}

// ─────────────────────────────────────────────────────────────────────────────
// onGpioChange — llamado por pigpio en su hilo interno
// Mide el ancho del pulso: tiempo entre flanco de subida y bajada
// ─────────────────────────────────────────────────────────────────────────────
void RcReceiverNode::onGpioChange(unsigned gpio, unsigned level, uint32_t tick)
{
  // level=1 → flanco de subida (inicio del pulso)
  // level=0 → flanco de bajada (fin del pulso)
  // level=2 → watchdog pigpio (ignorar)
  if (level == 2) { return; }

  const bool is_throttle = (static_cast<int>(gpio) == throttle_pin_);

  if (level == 1) {
    // Guardar tick del flanco de subida
    if (is_throttle) {
      throttle_rise_tick_ = tick;
    } else {
      steering_rise_tick_ = tick;
    }
  } else {
    // Flanco de bajada: calcular ancho del pulso
    // tick es uint32 y puede hacer overflow cada ~72 min → resta mod 2^32 es correcta
    uint32_t rise_tick = is_throttle ? throttle_rise_tick_ : steering_rise_tick_;
    int pulse_us = static_cast<int>(tick - rise_tick);  // diferencia en µs

    // Filtrar pulsos fuera de rango (ruido o glitches)
    if (pulse_us < 500 || pulse_us > 2500) { return; }

    if (is_throttle) {
      throttle_pulse_us_.store(pulse_us, std::memory_order_relaxed);
      throttle_valid_.store(true, std::memory_order_relaxed);
      last_throttle_time_ = this->now();
    } else {
      steering_pulse_us_.store(pulse_us, std::memory_order_relaxed);
      steering_valid_.store(true, std::memory_order_relaxed);
      last_steering_time_ = this->now();
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// publishTimerCallback — publica /cmd_vel a la tasa configurada
// ─────────────────────────────────────────────────────────────────────────────
void RcReceiverNode::publishTimerCallback()
{
  const rclcpp::Time now = this->now();

  const double throttle_age = (now - last_throttle_time_).seconds();
  const double steering_age = (now - last_steering_time_).seconds();

  const bool throttle_ok = throttle_valid_.load() && (throttle_age < signal_timeout_);
  const bool steering_ok  = steering_valid_.load()  && (steering_age  < signal_timeout_);

  // Si no hay señal válida → publicar cero y avisar
  if (!throttle_ok && !steering_ok) {
    static bool warned = false;
    if (!warned) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Sin señal RC. Comprueba las conexiones en GPIO %d y %d.",
        throttle_pin_, steering_pin_);
      warned = true;
    }
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  const int thr_us  = throttle_pulse_us_.load(std::memory_order_relaxed);
  const int str_us  = steering_pulse_us_.load(std::memory_order_relaxed);

  // Convertir a normalizado [-1, 1]
  double throttle_norm = pulseToNormalized(thr_us, pwm_min_us_, pwm_mid_us_, pwm_max_us_);
  double steering_norm  = pulseToNormalized(str_us,  pwm_min_us_, pwm_mid_us_, pwm_max_us_);

  // Aplicar zona muerta
  const double deadband_norm =
    static_cast<double>(deadband_us_) / (pwm_max_us_ - pwm_mid_us_);
  throttle_norm = applyDeadband(throttle_norm, deadband_norm);
  steering_norm  = applyDeadband(steering_norm,  deadband_norm);

  // Inversión opcional
  if (throttle_inverted_) { throttle_norm = -throttle_norm; }
  if (steering_inverted_)  { steering_norm  = -steering_norm;  }

  // Escalar a velocidades reales
  geometry_msgs::msg::Twist twist;
  twist.linear.x  = throttle_norm * max_linear_speed_;
  twist.angular.z = steering_norm  * max_angular_speed_;
  cmd_vel_pub_->publish(twist);

  // Publicar PWM crudo para diagnóstico
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

// ─────────────────────────────────────────────────────────────────────────────
// pulseToNormalized — mapea pulso µs a [-1, 1]
// ─────────────────────────────────────────────────────────────────────────────
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
// applyDeadband — fuerza a 0 si el valor está cerca del neutro
// ─────────────────────────────────────────────────────────────────────────────
double RcReceiverNode::applyDeadband(double value, double deadband_norm)
{
  if (std::abs(value) < deadband_norm) { return 0.0; }
  // Rescalar para que la salida empiece en 0 al salir de la zona muerta
  const double sign = (value > 0.0) ? 1.0 : -1.0;
  return sign * (std::abs(value) - deadband_norm) / (1.0 - deadband_norm);
}

double RcReceiverNode::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

}  // namespace rc_receiver

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rc_receiver::RcReceiverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}