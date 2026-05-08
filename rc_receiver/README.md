# rc_receiver - RC receiver PWM reader for ROS2 Humble

ROS2 C++ node that reads PWM signals from a standard RC receiver connected to the
Raspberry Pi GPIO pins and publishes them as `geometry_msgs/Twist` on `/cmd_vel`.

It uses **pigpio** edge callbacks to measure pulse widths with microsecond precision,
applies a deadband around neutral, and includes a signal timeout that publishes a
zero-velocity command when the RC transmitter is off or out of range.

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Publication | Velocity command from RC input |
| `/rc_receiver/raw_pwm` | `std_msgs/Float32MultiArray` | Publication | Raw PWM values for diagnostics |

The `raw_pwm` array has 4 elements: `[throttle_us, steering_us, throttle_norm, steering_norm]`.

`linear.x` carries the throttle channel, `angular.z` carries the steering channel.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `throttle_pin` | `13` | GPIO input pin for the throttle channel (BCM) |
| `steering_pin` | `17` | GPIO input pin for the steering channel (BCM) |
| `pwm_min_us` | `1000` | Minimum expected pulse width (µs) |
| `pwm_mid_us` | `1500` | Neutral pulse width (µs) |
| `pwm_max_us` | `2000` | Maximum expected pulse width (µs) |
| `deadband_us` | `30` | Deadband around neutral — pulses within this range are treated as zero |
| `max_linear_speed` | `1.0` | Full-throttle value published on `linear.x` |
| `max_angular_speed` | `1.0` | Full-steering value published on `angular.z` |
| `throttle_inverted` | `false` | Invert throttle channel |
| `steering_inverted` | `false` | Invert steering channel |
| `publish_rate` | `50.0` | Rate at which `/cmd_vel` is published (Hz) |
| `signal_timeout` | `0.5` | Seconds without a valid pulse before publishing zero velocity |

All parameters can be tuned in [config/params.yaml](config/params.yaml).

## Dependencies

### System — pigpio

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
cp -r rc_receiver ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select rc_receiver
source install/setup.bash
```

## Usage

### 1. Start the pigpio daemon

```bash
sudo pigpiod
```

To start it automatically on boot:

```bash
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### 2. Wire the RC receiver

Connect the signal wires of each channel to the configured GPIO pins.
The receiver must share ground with the Raspberry Pi.
Power the receiver from its own BEC or the ESC BEC — **not** from the Pi 3.3 V pin.

| RC channel | Signal wire | GPIO (BCM) | Physical pin |
|---|---|---|---|
| Throttle | GPIO 13 | 13 | 33 |
| Steering | GPIO 17 | 17 | 11 |

### 3. Launch

**Option A — launch file (recommended)**

Loads parameters from `config/params.yaml` and allows overriding the pins:

```bash
ros2 launch rc_receiver rc_receiver.launch.py

# Override pins
ros2 launch rc_receiver rc_receiver.launch.py \
  throttle_pin:=13 \
  steering_pin:=17
```

**Option B — direct node**

```bash
ros2 run rc_receiver rc_receiver_node \
  --ros-args --params-file ~/ros2_ws/install/rc_receiver/share/rc_receiver/config/params.yaml
```

## Monitor output

```bash
# Velocity commands
ros2 topic echo /cmd_vel

# Raw PWM values (µs and normalised)
ros2 topic echo /rc_receiver/raw_pwm
# data: [throttle_us, steering_us, throttle_norm, steering_norm]

# Publish rate
ros2 topic hz /cmd_vel
```

## Signal timeout

If no valid pulse is received on either channel within `signal_timeout` seconds,
the node logs a warning and publishes a zero `Twist`. This handles the case where
the transmitter is switched off or moves out of range, preventing the car from
continuing at the last known command.

## Calibration

If the neutral position drifts or the full-range does not match, adjust
`pwm_min_us`, `pwm_mid_us`, and `pwm_max_us` in `config/params.yaml` to match
the actual values output by your specific RC receiver. Use `raw_pwm` to read
the real pulse widths while moving the sticks.
