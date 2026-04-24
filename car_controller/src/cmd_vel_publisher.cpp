#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
/// @brief Nodo de demo que publica una secuencia en /cmd_vel
// ─────────────────────────────────────────────────────────────────────────────
class CmdVelPublisher : public rclcpp::Node
{
public:
  CmdVelPublisher()
  : Node("cmd_vel_publisher")
  {
    this->declare_parameter<double>("linear_speed",  0.5);
    this->declare_parameter<double>("angular_speed", 0.5);
    this->declare_parameter<double>("publish_rate",  10.0);

    const double lin  = this->get_parameter("linear_speed").as_double();
    const double ang  = this->get_parameter("angular_speed").as_double();
    const double rate = this->get_parameter("publish_rate").as_double();

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Secuencia: {duración_s, linear_x, angular_z}
    sequence_ = {
      {2.0,  lin,  0.0},   // avance recto
      {2.0,  lin,  ang},   // giro derecha
      {2.0,  lin, -ang},   // giro izquierda
      {1.0,  0.0,  0.0},   // parada
    };

    seq_start_ = this->now();

    const auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelPublisher::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "CmdVelPublisher iniciado (secuencia demo).");
  }

private:
  struct Step { double duration; double linear; double angular; };

  void timerCallback()
  {
    if (seq_idx_ >= sequence_.size()) {
      sendTwist(0.0, 0.0);
      return;
    }

    const auto & step    = sequence_[seq_idx_];
    const double elapsed = (this->now() - seq_start_).seconds();

    if (elapsed >= step.duration) {
      ++seq_idx_;
      seq_start_ = this->now();
      return;
    }

    sendTwist(step.linear, step.angular);
  }

  void sendTwist(double linear, double angular)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = linear;
    msg.angular.z = angular;
    pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Step> sequence_;
  std::size_t seq_idx_{0};
  rclcpp::Time seq_start_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelPublisher>());
  rclcpp::shutdown();
  return 0;
}