#!/usr/bin/env python3
"""
coral_test_node.py

Nodo ROS2 simple para verificar que la Google Coral (Edge TPU) funciona
correctamente desde dentro de un contenedor Docker con ROS2 Humble.

Pasos que realiza:
  1. Detecta el dispositivo Coral (USB o PCIe/M.2)
  2. Descarga el modelo de prueba oficial de la Coral si no existe
  3. Ejecuta una inferencia de prueba con datos aleatorios
  4. Publica el resultado en /coral_test_result (std_msgs/String)
  5. Imprime un diagnóstico claro en consola

Uso:
  ros2 run coral_test coral_test_node
"""

import os
import sys
import time
import urllib.request
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ctypes

# ------------------------------------------------------------------
# Rutas y URL del modelo de prueba oficial de la Coral
# ------------------------------------------------------------------
MODEL_URL = (
    "https://github.com/google-coral/test_data/raw/master/"
    "mobilenet_v2_1.0_224_quant_edgetpu.tflite"
)
MODEL_DIR = os.path.expanduser("~/coral_models")
MODEL_PATH = os.path.join(MODEL_DIR, "mobilenet_v2_1.0_224_quant_edgetpu.tflite")

# Entrada esperada por MobileNet V2 224
INPUT_SHAPE = (1, 224, 224, 3)


class CoralTestNode(Node):
    """Nodo ROS2 que verifica el funcionamiento de la Google Coral."""

    def __init__(self):
        super().__init__('coral_test_node')
        self.publisher_ = self.create_publisher(String, 'coral_test_result', 10)
        self.get_logger().info('=== Nodo de prueba Google Coral iniciado ===')

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------

    def _download_model(self):
        """Descarga el modelo de prueba si no está presente."""
        if os.path.exists(MODEL_PATH):
            self.get_logger().info(f'Modelo ya existe: {MODEL_PATH}')
            return True

        self.get_logger().info(f'Descargando modelo de prueba desde:\n  {MODEL_URL}')
        os.makedirs(MODEL_DIR, exist_ok=True)
        try:
            urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
            self.get_logger().info('Modelo descargado correctamente.')
            return True
        except Exception as e:
            self.get_logger().error(f'Error al descargar el modelo: {e}')
            self.get_logger().warn(
                'Descarga manual:\n'
                f'  mkdir -p {MODEL_DIR}\n'
                f'  wget -O {MODEL_PATH} \\\n'
                f'    "{MODEL_URL}"'
            )
            return False

    def _detect_coral_device(self):
        """Detecta si hay un dispositivo Coral disponible."""
        # Coral USB Accelerator
        usb_path = '/dev/bus/usb'
        # Coral PCIe / M.2 aparece como apex_0
        pcie_path = '/dev/apex_0'

        if os.path.exists(pcie_path):
            return 'PCIe/M.2', pcie_path
        if os.path.exists(usb_path):
            # Intento básico de detección por vendor ID (18d1 = Google)
            try:
                result = os.popen('lsusb 2>/dev/null | grep -i "18d1:9302\\|1a6e:089a"').read()
                if result.strip():
                    return 'USB', result.strip()
            except Exception:
                pass
            return 'USB (posible)', usb_path

        return None, None

    # ------------------------------------------------------------------
    # Test principal
    # ------------------------------------------------------------------

    def run_test(self):
        msg = String()

        # ── 1. Importar pycoral ──────────────────────────────────────
        self.get_logger().info('Verificando librería pycoral...')
        try:
            from pycoral.utils.edgetpu import make_interpreter
            from pycoral.adapters import common as coral_common
        except ImportError:
            error = (
                'pycoral NO está instalado.\n'
                'Instálalo con:\n'
                '  pip3 install pycoral\n'
                'O sigue: https://coral.ai/software/'
            )
            self.get_logger().error(error)
            msg.data = f'[FALLO] {error}'
            self.publisher_.publish(msg)
            return False

        self.get_logger().info('pycoral encontrado ✓')

        # ── 2. Detectar dispositivo ──────────────────────────────────
        dev_type, dev_path = self._detect_coral_device()
        if dev_type:
            self.get_logger().info(f'Dispositivo Coral detectado: {dev_type}  ({dev_path})')
        else:
            self.get_logger().warn(
                'No se detectó dispositivo Coral en /dev/apex_0 ni /dev/bus/usb.\n'
                'Asegúrate de que:\n'
                '  - El dispositivo esté conectado\n'
                '  - El contenedor Docker tenga acceso: --privileged o --device=/dev/...\n'
                '  - Los drivers estén instalados en el host'
            )

        # ── 3. Descargar modelo ──────────────────────────────────────
        if not self._download_model():
            msg.data = '[FALLO] No se pudo obtener el modelo de prueba.'
            self.publisher_.publish(msg)
            return False

        # ── 4. Cargar intérprete Edge TPU ────────────────────────────
        self.get_logger().info('Cargando modelo en la Edge TPU...')
        try:
            interpreter = make_interpreter(MODEL_PATH)
            interpreter.allocate_tensors()
        except Exception as e:
            error = (
                f'Error al cargar el modelo en la Edge TPU: {e}\n'
                'Posibles causas:\n'
                '  - El dispositivo Coral no está accesible desde Docker\n'
                '    Añade --privileged o --device=/dev/apex_0 al docker run\n'
                '  - Drivers no instalados en el host Raspberry Pi'
            )
            self.get_logger().error(error)
            msg.data = f'[FALLO] {error}'
            self.publisher_.publish(msg)
            return False

        self.get_logger().info('Modelo cargado en Edge TPU ✓')

        # ── 5. Inferencia de prueba ──────────────────────────────────
        self.get_logger().info('Ejecutando inferencia de prueba con datos aleatorios...')
        try:
            input_details = interpreter.get_input_details()
            output_details = interpreter.get_output_details()

            # Datos aleatorios uint8 con la forma correcta
            fake_input = np.random.randint(0, 255, INPUT_SHAPE, dtype=np.uint8)
            coral_common.set_input(interpreter, fake_input)

            t0 = time.perf_counter()
            interpreter.invoke()
            t1 = time.perf_counter()
            latency_ms = (t1 - t0) * 1000

            output_data = coral_common.output_tensor(interpreter, 0)
            top_class = int(np.argmax(output_data))

            result = (
                f'[OK] Google Coral funciona correctamente.\n'
                f'  Dispositivo : {dev_type or "desconocido"}\n'
                f'  Modelo      : mobilenet_v2_1.0_224_quant_edgetpu.tflite\n'
                f'  Latencia    : {latency_ms:.2f} ms\n'
                f'  Clase top-1 : {top_class} (entrada aleatoria, clase sin sentido es normal)\n'
                f'  Input shape : {input_details[0]["shape"].tolist()}\n'
                f'  Output shape: {output_details[0]["shape"].tolist()}'
            )
            self.get_logger().info(result)
            msg.data = result
            self.publisher_.publish(msg)
            libedgetpu = ctypes.CDLL("libedgetpu.so.1", mode=ctypes.RTLD_GLOBAL)
            return True

        except Exception as e:
            error = f'Error durante la inferencia: {e}'
            self.get_logger().error(error)
            msg.data = f'[FALLO] {error}'
            self.publisher_.publish(msg)
            return False


# ------------------------------------------------------------------
# Entrypoint
# ------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CoralTestNode()

    try:
        success = node.run_test()
        # Spin brevemente para que el publisher entregue el mensaje
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
