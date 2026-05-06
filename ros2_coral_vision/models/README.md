# Directorio de modelos

Coloca aquí tus modelos TFLite compilados para Edge TPU.

## Descargar modelos

```bash
# SSD MobileNet V2 - Detección de objetos COCO
wget https://github.com/google-coral/test_data/raw/master/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite
wget https://raw.githubusercontent.com/google-coral/test_data/master/coco_labels.txt

# EfficientNet-M - Clasificación ImageNet
wget https://github.com/google-coral/test_data/raw/master/efficientnet-edgetpu-M_quant_edgetpu.tflite
wget https://raw.githubusercontent.com/google-coral/test_data/master/imagenet_labels.txt
```

## Compilar tu propio modelo

Si tienes un modelo TFLite propio, puedes compilarlo para Edge TPU:

```bash
# Instalar el compilador Edge TPU
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
sudo apt update && sudo apt install edgetpu-compiler

# Compilar
edgetpu_compiler mi_modelo.tflite
# Genera: mi_modelo_edgetpu.tflite
```
