# car_controller - RC car PWM control with ROS2 Humble

ROS2 C++ node that translates `geometry_msgs/Twist` velocity commands into PWM signals
for a servo (steering) and an ESC (throttle) on a Raspberry Pi, using the **pigpio** daemon.

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscription | Velocity command |
| `/car_controller/pwm_state` | `std_msgs/Float32MultiArray` | Publication | Current PWM state |

The `pwm_state` array has 4 elements in order: `[steering_us, throttle_us, steering_norm, throttle_norm]`.

`linear.x` controls throttle, `angular.z` controls steering.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `steering_pin` | `12` | GPIO pin for the steering servo (hardware PWM) |
| `throttle_pin` | `19` | GPIO pin for the throttle ESC (hardware PWM) |
| `pwm_min_us` | `1000` | Minimum PWM pulse width (µs) |
| `pwm_neutral_us` | `1500` | Neutral PWM pulse width (µs) |
| `pwm_max_us` | `2000` | Maximum PWM pulse width (µs) |
| `max_linear_speed` | `1.0` | Speed value that maps to full throttle|
| `max_angular_speed` | `1.0` | Angular speed value that maps to full steering|
| `steering_inverted` | `false` | Invert steering direction |
| `throttle_inverted` | `false` | Invert throttle direction |
| `cmd_vel_timeout` | `0.5` | Seconds without a command before the car stops (watchdog) |

All parameters can be tuned in [config/params.yaml](config/params.yaml).

## Dependencies

### System — pigpio

The node uses `pigpiod_if2` to communicate with the pigpio daemon over a socket
(works both natively and inside Docker).

```bash
# Install pigpio
sudo apt install pigpio python3-pigpio

# Or build from source
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip && cd pigpio-master
make && sudo make install
```

### ROS2 packages

```bash
sudo apt install \
  ros-humble-rclcpp \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs
```

## Build

```bash
cp -r car_controller ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select car_controller
source install/setup.bash
```

## Usage

### 1. Start the pigpio daemon

The daemon must be running on the host before launching the node.

```bash
sudo pigpiod
```

To start it automatically on boot:

```bash
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### 2. Launch

**Option A — launch file (recommended)**

Loads parameters from `config/params.yaml` and allows overriding pins and timeout:

```bash
ros2 launch car_controller car_controller.launch.py

# Override pins or timeout
ros2 launch car_controller car_controller.launch.py \
  steering_pin:=12 \
  throttle_pin:=19 \
  cmd_vel_timeout:=1.0
```

**Option B — direct node**

```bash
ros2 run car_controller car_controller_node \
  --ros-args --params-file ~/ros2_ws/install/car_controller/share/car_controller/config/params.yaml
```

### 3. Send a command

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Stop immediately
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Monitor PWM state

```bash
ros2 topic echo /car_controller/pwm_state
# data: [steering_us, throttle_us, steering_norm, throttle_norm]
```

## Watchdog

If no `/cmd_vel` message is received within `cmd_vel_timeout` seconds, both channels
are reset to neutral (1500 µs) to stop the car. This protects against node crashes
or lost connections.

## Hardware wiring (Raspberry Pi 4)

| Signal | GPIO (BCM) | Physical pin |
|---|---|---|
| Steering servo | 12 | 32 |
| Throttle ESC | 19 | 35 |

Both pins support hardware PWM on the Raspberry Pi 4.
Connect signal wires to the GPIO pins; power the servo and ESC from an independent BEC.
