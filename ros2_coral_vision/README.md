# ROS2 + Google Coral Edge TPU en Raspberry Pi 4
### Ubuntu 22.04 · Python 3.10 · ROS2 Humble · Coral USB Accelerator

---

## Requisitos

- Raspberry Pi 4 con Ubuntu 22.04 aarch64
- ROS2 Humble instalado
- Google Coral USB Accelerator conectado

---

## Paso 1 — Añadir el repositorio de Google Coral

```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | \
  sudo tee /etc/apt/sources.list.d/coral-edgetpu.list

curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt update
```

---

## Paso 2 — Instalar libedgetpu

La versión oficial de Google no es compatible con Python 3.10. Hay que usar el fork actualizado de feranick:

```bash
wget https://github.com/feranick/libedgetpu/releases/download/16.0TF2.17.1-1/libedgetpu1-std_16.0tf2.17.1-1.ubuntu22.04_arm64.deb

sudo dpkg -i libedgetpu1-std_16.0tf2.17.1-1.ubuntu22.04_arm64.deb
sudo ldconfig
```

---

## Paso 3 — Instalar pycoral y el runtime

El paquete oficial de apt no soporta Python 3.10. Instalar desde pip en el Python del sistema:

```bash
sudo pip install \
  ai-edge-litert \
  https://github.com/oberluz/pycoral/releases/download/2.13.0/pycoral-2.13.0-cp310-cp310-linux_aarch64.whl \
  --break-system-packages
```

---

## Paso 4 — Descargar modelos

```bash
mkdir -p ~/modelos_coral

# Modelo para Edge TPU
wget -P ~/modelos_coral \
  https://github.com/google-coral/test_data/raw/master/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite

# Etiquetas COCO (80 clases)
wget -P ~/modelos_coral \
  https://raw.githubusercontent.com/google-coral/test_data/master/coco_labels.txt
```

---

## Paso 5 — Instalar el paquete ROS2

```bash
cp -r ros2_coral_vision ~/ros2_ws/src/

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ros2_coral_vision
source install/setup.bash
```

Opcional, para no tener que hacer `source` cada vez:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Paso 6 — Verificar que el Edge TPU funciona

```bash
python3 -c "
from pycoral.utils.edgetpu import make_interpreter
interp = make_interpreter('/root/modelos_coral/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite')
interp.allocate_tensors()
print('✓ Edge TPU OK')
"
```

---

## Paso 7 — Lanzar el nodo

```bash
ros2 launch ros2_coral_vision detection.launch.py \
  model_path:=/root/modelos_coral/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite \
  labels_path:=/root/modelos_coral/coco_labels.txt \
  use_edgetpu:=true
```

Resultado esperado:

```
[coral_vision_node-1] [INFO]: ✓ CoralVisionNode iniciado
[coral_vision_node-1] [INFO]: Stats → frames=116 | latencia_media=49.6 ms | FPS≈20.2
```

---

## Topics

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Entrada de imágenes |
| `/coral/detections` | `vision_msgs/Detection2DArray` | Detecciones con bbox y score |
| `/coral/image_annotated` | `sensor_msgs/Image` | Imagen con anotaciones |

---

## Versiones utilizadas

| Componente | Versión |
|------------|---------|
| OS | Ubuntu 22.04 aarch64 |
| Python | 3.10 |
| ROS2 | Humble |
| libedgetpu | 16.0tf2.17.1-1 (fork feranick) |
| pycoral | 2.13.0 (fork oberluz) |
| ai-edge-litert | última en PyPI |