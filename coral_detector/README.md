# coral_detector - Object detection with Google Coral Edge TPU and ROS2 Humble

ROS2 Python node that receives camera images, runs object detection using a
SSD MobileNet V2 model trained on COCO (80 classes), and publishes the results.

It tries to run inference on the **Google Coral Edge TPU** and falls back automatically
to CPU+XNNPACK (4 threads) if the TPU is unavailable or its driver has a bug.
The model and labels are downloaded automatically on the first run.

Inspired by [coral_usb_ros](https://github.com/jsk-ros-pkg/coral_usb_ros).

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Subscription | Input camera image |
| `/coral/detections` | `vision_msgs/Detection2DArray` | Publication | Detections with bbox, class and score |
| `/coral/image_detections` | `sensor_msgs/Image` | Publication | Annotated image with bounding boxes |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `model_path` | `~/coral_models/ssd_mobilenet_v2_coco_quant_postprocess.tflite` | Path to the .tflite model |
| `labels_path` | `~/coral_models/coco_labels.txt` | Path to the labels file |
| `score_threshold` | `0.40` | Minimum confidence score to publish a detection (0.0 - 1.0) |
| `max_detections` | `10` | Maximum number of detections per frame |
| `publish_image` | `true` | Publish annotated image on `/coral/image_detections` |
| `use_tpu` | `false` | Attempt to run inference on the Edge TPU |

## Dependencies

### Python packages

```bash
pip install tflite-runtime numpy

# Optional but recommended (annotated image output)
pip install opencv-python
sudo apt install ros-humble-cv-bridge
```

### ROS2 packages

```bash
sudo apt install \
  ros-humble-vision-msgs \
  ros-humble-sensor-msgs \
  ros-humble-cv-bridge
```

### Edge TPU (optional)

Only needed if `use_tpu: true`. Follow the official Coral setup guide to install
`libedgetpu` and `pycoral` for your platform.

> **Note:** libedgetpu 16.0 on Debian Trixie has a known bug with
> `_postprocess_edgetpu.tflite` models. The node handles this automatically
> by falling back to CPU.

## Build

```bash
cp -r coral_detector ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select coral_detector
source install/setup.bash
```

## Usage

### Option A — launch file (recommended)

```bash
ros2 launch coral_detector coral_detector.launch.py

# With custom parameters
ros2 launch coral_detector coral_detector.launch.py \
  score_threshold:=0.5 \
  max_detections:=5 \
  image_topic:=/camera/color/image_raw
```

### Option B — direct node

```bash
ros2 run coral_detector detector_node \
  --ros-args \
  -p score_threshold:=0.5 \
  -p max_detections:=5 \
  -p use_tpu:=true
```

### Remap to a different camera topic

```bash
ros2 run coral_detector detector_node \
  --ros-args --remap /camera/image_raw:=/your/camera/topic
```

## Monitor output

```bash
# Print detections as text
ros2 topic echo /coral/detections

# View annotated image in RViz2
rviz2
# -> Add -> By topic -> /coral/image_detections -> Image

# Approximate FPS
ros2 topic hz /coral/detections
```

## Detectable COCO classes (80 objects)

person, bicycle, car, motorcycle, airplane, bus, train, truck, boat,
traffic light, fire hydrant, stop sign, parking meter, bench, bird,
cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe, backpack,
umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports ball,
kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket,
bottle, wine glass, cup, fork, knife, spoon, bowl, banana, apple,
sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake, chair,
couch, potted plant, bed, dining table, toilet, tv, laptop, mouse,
remote, keyboard, cell phone, microwave, oven, toaster, sink,
refrigerator, book, clock, vase, scissors, teddy bear, hair drier, toothbrush
