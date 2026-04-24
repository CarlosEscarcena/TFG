#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

static constexpr double LINEAR_STEP  = 0.1;
static constexpr double ANGULAR_STEP = 0.1;
static constexpr double MAX_LINEAR   = 1.0;
static constexpr double MAX_ANGULAR  = 1.0;

static const char * BANNER = R"(
╔══════════════════════════════════════╗
║        CAR KEYBOARD TELEOP          ║
╠══════════════════════════════════════╣
║  W / S  →  acelerar / frenar        ║
║  A / D  →  izquierda / derecha      ║
║  SPACE  →  parada de emergencia     ║
║  Q      →  salir                    ║
╚══════════════════════════════════════╝
)";

// ─────────────────────────────────────────────────────────────────────────────
/// Lee un carácter sin necesidad de pulsar Enter
// ─────────────────────────────────────────────────────────────────────────────
char getKey()
{
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  char ch = 0;
  if (read(STDIN_FILENO, &ch, 1) < 0) { ch = 0; }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

// ─────────────────────────────────────────────────────────────────────────────
class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("keyboard_teleop")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    std::puts(BANNER);
  }

  void run()
  {
    while (rclcpp::ok()) {
      char key = static_cast<char>(std::tolower(getKey()));

      switch (key) {
        case 'w':
          linear_  = std::min(linear_  + LINEAR_STEP,  MAX_LINEAR);
          break;
        case 's':
          linear_  = std::max(linear_  - LINEAR_STEP, -MAX_LINEAR);
          break;
        case 'a':
          angular_ = std::min(angular_ + ANGULAR_STEP, MAX_ANGULAR);
          break;
        case 'd':
          angular_ = std::max(angular_ - ANGULAR_STEP, -MAX_ANGULAR);
          break;
        case ' ':
          linear_  = 0.0;
          angular_ = 0.0;
          std::puts("\n⛔ PARADA DE EMERGENCIA");
          break;
        case 'q':
          std::puts("\nSaliendo...");
          publishTwist(0.0, 0.0);
          return;
        default:
          continue;
      }

      publishTwist(linear_, angular_);
      std::printf(
        "\r  linear.x=%+.1f  angular.z=%+.1f    ",
        linear_, angular_
      );
      std::fflush(stdout);
    }
  }

private:
  void publishTwist(double linear, double angular)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = linear;
    msg.angular.z = angular;
    pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  double linear_{0.0};
  double angular_{0.0};
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  node->run();
  rclcpp::shutdown();
  return 0;
}