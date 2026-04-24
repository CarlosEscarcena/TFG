# coral_detector — Detección de objetos con Google Coral + ROS2 Humble

Nodo ROS2 que recibe imágenes de `/image_raw`, las procesa en la **Google Coral Edge TPU**
con un modelo SSD MobileNet V2 entrenado en COCO (80 clases), y publica las detecciones.

## Topics

| Topic | Tipo | Descripción |
|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | **Entrada** — imagen de la cámara |
| `/coral/detections` | `vision_msgs/Detection2DArray` | Detecciones con bbox, clase y score |
| `/coral/image_detections` | `sensor_msgs/Image` | Imagen anotada con cajas y etiquetas |

## Parámetros

| Parámetro | Default | Descripción |
|---|---|---|
| `model_path` | `~/coral_models/ssd_mobilenet_v2_...edgetpu.tflite` | Ruta al modelo |
| `labels_path` | `~/coral_models/coco_labels.txt` | Ruta a las etiquetas |
| `score_threshold` | `0.40` | Confianza mínima (0.0 – 1.0) |
| `max_detections` | `10` | Máximo de objetos por frame |
| `publish_image` | `true` | Publicar imagen anotada |

## Instalación

### Dependencias
```bash
# cv_bridge y vision_msgs
apt install ros-humble-cv-bridge ros-humble-vision-msgs python3-opencv
```

### Compilar
```bash
cp -r coral_detector ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select coral_detector
source install/setup.bash
```

El modelo y las etiquetas se descargan automáticamente la primera vez.

## Uso

### Opción 1 — launch file (recomendado)
```bash
ros2 launch coral_detector coral_detector.launch.py

# Con parámetros personalizados
ros2 launch coral_detector coral_detector.launch.py \
  score_threshold:=0.5 \
  image_topic:=/camera/image_raw
```

### Opción 2 — nodo directo
```bash
ros2 run coral_detector detector_node \
  --ros-args \
  -p score_threshold:=0.5 \
  -p max_detections:=5
```

### Redirigir a otro topic de cámara
```bash
ros2 run coral_detector detector_node \
  --ros-args --remap /image_raw:=/camera/color/image_raw
```

## Ver resultados

```bash
# Ver detecciones en texto
ros2 topic echo /coral/detections

# Ver imagen anotada en RViz2
rviz2
# → Add → By topic → /coral/image_detections → Image

# Ver FPS aproximado
ros2 topic hz /coral/detections
```

## Clases COCO detectables (80 objetos)

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
