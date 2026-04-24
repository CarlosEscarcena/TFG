# coral_test — Paquete ROS2 para verificar la Google Coral

Paquete mínimo para comprobar que la **Google Coral Edge TPU** funciona
correctamente desde un contenedor Docker con **ROS2 Humble** en una Raspberry Pi 4.

---

## Requisitos previos en el HOST (Raspberry Pi 4)

### 1. Drivers de la Coral
```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" \
  | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt update
sudo apt install libedgetpu1-std   # velocidad estándar (recomendado)
# sudo apt install libedgetpu1-max  # máxima velocidad (calienta más)
```

### 2. Verificar que el host ve la Coral USB
```bash
lsusb | grep -i google   # debe aparecer "18d1:9302" o "1a6e:089a"
```

---

## Dentro del contenedor Docker

### 1. Instalar pycoral y tflite-runtime
```bash
pip3 install --extra-index-url https://google-coral.github.io/py-repo/ \
    pycoral tflite-runtime
```

### 2. Asegurarse de que el contenedor tiene acceso al dispositivo
Al lanzar el contenedor, añade uno de estos flags:

```bash
# Opción A – acceso completo (más fácil para pruebas)
docker run --privileged ...

# Opción B – solo el dispositivo Coral USB
docker run --device=/dev/bus/usb ...

# Opción B – solo la Coral PCIe/M.2
docker run --device=/dev/apex_0 ...
```

---

## Compilar y ejecutar el paquete

```bash
# Copiar el paquete a tu workspace de ROS2 (dentro del contenedor)
cp -r coral_test ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select coral_test
source install/setup.bash

# Ejecutar el nodo de prueba
ros2 run coral_test coral_test_node
```

### Salida esperada si TODO funciona
```
[INFO] === Nodo de prueba Google Coral iniciado ===
[INFO] pycoral encontrado ✓
[INFO] Dispositivo Coral detectado: USB  (...)
[INFO] Modelo ya existe: ~/coral_models/mobilenet_v2_1.0_224_quant_edgetpu.tflite
[INFO] Cargando modelo en la Edge TPU...
[INFO] Modelo cargado en Edge TPU ✓
[INFO] Ejecutando inferencia de prueba con datos aleatorios...
[INFO] [OK] Google Coral funciona correctamente.
         Dispositivo : USB
         Modelo      : mobilenet_v2_1.0_224_quant_edgetpu.tflite
         Latencia    : ~2-5 ms
         Clase top-1 : XX  (aleatoria, es normal)
```

### Ver el resultado por topic
```bash
ros2 topic echo /coral_test_result
```

---

## Solución de problemas comunes

| Síntoma | Causa probable | Solución |
|---------|---------------|----------|
| `ImportError: pycoral` | pycoral no instalado | `pip3 install pycoral` (ver arriba) |
| `Failed to load delegate` | Coral no accesible | Añadir `--privileged` al docker run |
| `No se detectó dispositivo` | Drivers no instalados en host | Instalar `libedgetpu1-std` en la RPi |
| Latencia > 100 ms | Fallback a CPU (sin TPU) | Revisar acceso al dispositivo |
