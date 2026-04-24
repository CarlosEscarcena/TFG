#!/usr/bin/env python3
"""
detector_node.py

Nodo ROS2 que detecta objetos en tiempo real usando la Google Coral Edge TPU.

Dado que libedgetpu 16.0 en Debian Trixie tiene un bug con los modelos
_postprocess_edgetpu.tflite, este nodo intenta cargar el modelo en la TPU
y si falla hace fallback automático al modelo base corriendo en CPU+XNNPACK
(4 hilos). El resultado es idéntico en ambos casos.

Subscripciones:
  /image_raw  (sensor_msgs/Image)

Publicaciones:
  /coral/detections        (vision_msgs/Detection2DArray)
  /coral/image_detections  (sensor_msgs/Image)

Parámetros:
  model_path      : ruta al modelo .tflite
  labels_path     : ruta al fichero de etiquetas
  score_threshold : umbral mínimo de confianza (default: 0.40)
  max_detections  : máximo de detecciones      (default: 10)
  publish_image   : publicar imagen anotada    (default: true)
  use_tpu         : intentar usar Edge TPU     (default: true)
"""

import ctypes
import os
import urllib.request

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

try:
    from cv_bridge import CvBridge
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# ──────────────────────────────────────────────────────────────────
# Assets
# ──────────────────────────────────────────────────────────────────
MODEL_CPU_URL = (
    "https://github.com/google-coral/edgetpu/raw/master/test_data/"
    "ssd_mobilenet_v2_coco_quant_postprocess.tflite"
)
MODEL_TPU_URL = (
    "https://raw.githubusercontent.com/google-coral/test_data/master/"
    "ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite"
)
LABELS_URL = (
    "https://raw.githubusercontent.com/google-coral/test_data/master/"
    "coco_labels.txt"
)

MODEL_DIR         = os.path.expanduser("~/coral_models")
DEFAULT_MODEL_CPU = os.path.join(MODEL_DIR, "ssd_mobilenet_v2_coco_quant_postprocess.tflite")
DEFAULT_MODEL_TPU = os.path.join(MODEL_DIR, "ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite")
DEFAULT_LABELS    = os.path.join(MODEL_DIR, "coco_labels.txt")

BOX_COLORS = [
    (0, 255, 0), (255, 80, 0), (0, 80, 255), (255, 0, 180),
    (0, 220, 220), (180, 0, 255), (255, 200, 0), (0, 128, 255),
]


# ──────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────

def _download(url: str, dest: str, logger) -> bool:
    if os.path.exists(dest):
        return True
    logger.info(f'Descargando {os.path.basename(dest)} ...')
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    try:
        urllib.request.urlretrieve(url, dest)
        logger.info(f'  → {dest}')
        return True
    except Exception as e:
        logger.error(f'Error descargando {url}: {e}')
        return False


def _load_labels(path: str) -> dict:
    labels = {}
    try:
        with open(path, 'r') as f:
            for idx, line in enumerate(f):
                line = line.strip()
                if not line:
                    continue
                parts = line.split(None, 1)
                if len(parts) == 2 and parts[0].isdigit():
                    labels[int(parts[0])] = parts[1]
                else:
                    labels[idx] = line
    except Exception:
        pass
    return labels


# ──────────────────────────────────────────────────────────────────
# Nodo
# ──────────────────────────────────────────────────────────────────

class CoralDetectorNode(Node):

    def __init__(self):
        super().__init__('coral_detector_node')

        # ── Parámetros ────────────────────────────────────────────
        self.declare_parameter('model_path',      DEFAULT_MODEL_CPU)
        self.declare_parameter('labels_path',     DEFAULT_LABELS)
        self.declare_parameter('score_threshold', 0.40)
        self.declare_parameter('max_detections',  10)
        self.declare_parameter('publish_image',   True)
        self.declare_parameter('use_tpu',         False)

        self.model_path      = self.get_parameter('model_path').value
        self.labels_path     = self.get_parameter('labels_path').value
        self.score_threshold = self.get_parameter('score_threshold').value
        self.max_detections  = self.get_parameter('max_detections').value
        self.publish_image   = self.get_parameter('publish_image').value
        self.use_tpu         = self.get_parameter('use_tpu').value

        # ── Publishers ────────────────────────────────────────────
        self.pub_detections = self.create_publisher(
            Detection2DArray, '/coral/detections', 10)

        self.pub_image = None
        if self.publish_image and CV2_AVAILABLE:
            self.pub_image = self.create_publisher(
                Image, '/coral/image_detections', 10)
            self.bridge = CvBridge()
        elif self.publish_image:
            self.get_logger().warn(
                'cv_bridge/opencv no disponibles. publish_image desactivado.')

        # ── Subscriber ────────────────────────────────────────────
        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10)

        # ── Inicializar intérprete ────────────────────────────────
        self.interpreter = None
        self.labels      = {}
        self.model_w     = 300
        self.model_h     = 300
        self.using_tpu   = False
        self._init_interpreter()

        self.get_logger().info(
            'Coral Detector listo. Esperando imágenes en camera/image_raw ...')

    # ── Inicialización ────────────────────────────────────────────

    def _init_interpreter(self):
        # Etiquetas
        if not _download(LABELS_URL, self.labels_path, self.get_logger()):
            return
        self.labels = _load_labels(self.labels_path)
        self.get_logger().info(f'Etiquetas cargadas: {len(self.labels)} clases')

        # Modelo CPU siempre disponible como fallback
        if not _download(MODEL_CPU_URL, DEFAULT_MODEL_CPU, self.get_logger()):
            return

        # ── Intentar TPU ──────────────────────────────────────────
        if self.use_tpu:
            try:
                # ctypes.CDLL("libedgetpu.so.1", mode=ctypes.RTLD_GLOBAL)
                _download(MODEL_TPU_URL, DEFAULT_MODEL_TPU, self.get_logger())

                from pycoral.utils.edgetpu import make_interpreter

                self.get_logger().info('Intentando cargar modelo en Edge TPU...')
                interp = make_interpreter(DEFAULT_MODEL_TPU)
                interp.allocate_tensors()

                inp   = interp.get_input_details()
                h, w  = int(inp[0]['shape'][1]), int(inp[0]['shape'][2])
                dummy = np.zeros((h, w, 3), dtype=np.uint8)
                interp.set_tensor(inp[0]['index'], dummy.reshape(inp[0]['shape']))
                interp.invoke()

                self.interpreter = interp
                self.model_h     = h
                self.model_w     = w
                self.using_tpu   = True
                self.get_logger().info(
                    f'✓ Edge TPU activa | input: {w}x{h} | '
                    f'umbral: {self.score_threshold}')
                return

            except Exception as e:
                self.get_logger().warn(
                    f'Edge TPU no disponible ({e}). Usando CPU.')

        # ── CPU fallback ──────────────────────────────────────────
        self._load_cpu(DEFAULT_MODEL_CPU)

    def _load_cpu(self, model_path: str):
        try:
            import tflite_runtime.interpreter as tflite

            self.get_logger().info(
                f'Cargando modelo en CPU: {os.path.basename(model_path)}')
            interp = tflite.Interpreter(model_path, num_threads=4)
            interp.allocate_tensors()

            inp   = interp.get_input_details()
            h, w  = int(inp[0]['shape'][1]), int(inp[0]['shape'][2])
            dummy = np.zeros(inp[0]['shape'], dtype=np.uint8)
            interp.set_tensor(inp[0]['index'], dummy)
            interp.invoke()

            self.interpreter = interp
            self.model_h     = h
            self.model_w     = w
            self.using_tpu   = False
            self.get_logger().info(
                f'✓ CPU (XNNPACK 4 hilos) | input: {w}x{h} | '
                f'umbral: {self.score_threshold}')

        except Exception as e:
            self.get_logger().error(f'Error cargando modelo CPU: {e}')

    # ── Callback imagen ───────────────────────────────────────────

    def _image_callback(self, msg: Image):
        if self.interpreter is None:
            return

        # Convertir mensaje ROS a BGR
        try:
            if CV2_AVAILABLE:
                frame_bgr = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            else:
                data      = np.frombuffer(msg.data, dtype=np.uint8)
                frame_bgr = data.reshape((msg.height, msg.width, -1))
                if msg.encoding == 'rgb8':
                    frame_bgr = frame_bgr[:, :, ::-1]
        except Exception as e:
            self.get_logger().warn(f'Error convirtiendo imagen: {e}')
            return

        img_h, img_w = frame_bgr.shape[:2]

        # Redimensionar a 300x300 RGB
        if CV2_AVAILABLE:
            resized   = cv2.resize(frame_bgr, (self.model_w, self.model_h))
            rgb_input = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        else:
            from PIL import Image as PILImage
            pil       = PILImage.fromarray(frame_bgr[:, :, ::-1])
            pil       = pil.resize((self.model_w, self.model_h))
            rgb_input = np.array(pil)

        # ── Inferencia ────────────────────────────────────────────
        try:
            inp_details = self.interpreter.get_input_details()

            if self.using_tpu:
                from pycoral.adapters import common as coral_common
                coral_common.set_input(self.interpreter, rgb_input)
            else:
                self.interpreter.set_tensor(
                    inp_details[0]['index'],
                    rgb_input.reshape(inp_details[0]['shape']))

            self.interpreter.invoke()

        except Exception as e:
            self.get_logger().warn(f'Error en inferencia: {e}')
            return

        # ── Leer outputs ──────────────────────────────────────────
        # Formato postprocess:
        #   [0] boxes    (1, N, 4)  float32  [ymin,xmin,ymax,xmax] norm 0-1
        #   [1] classes  (1, N)     float32
        #   [2] scores   (1, N)     float32
        #   [3] num_det  (1,)       float32
        try:
            out = self.interpreter.get_output_details()

            if self.using_tpu:
                from pycoral.adapters import detect as coral_detect
                objects    = coral_detect.get_objects(
                    self.interpreter,
                    score_threshold=self.score_threshold)
                detections = self._parse_tpu(objects, img_w, img_h)
            else:
                boxes   = self.interpreter.get_tensor(out[0]['index'])[0]
                classes = self.interpreter.get_tensor(out[1]['index'])[0]
                scores  = self.interpreter.get_tensor(out[2]['index'])[0]
                num_det = int(self.interpreter.get_tensor(out[3]['index'])[0])
                detections = self._parse_cpu(
                    boxes, classes, scores, num_det, img_w, img_h)

        except Exception as e:
            self.get_logger().warn(f'Error leyendo outputs: {e}')
            return

        # ── Publicar Detection2DArray ─────────────────────────────
        det_array        = Detection2DArray()
        det_array.header = msg.header

        for class_id, score, x0, y0, x1, y1 in detections:
            det        = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x0 + x1) / 2.0
            det.bbox.center.position.y = (y0 + y1) / 2.0
            det.bbox.size_x            = float(x1 - x0)
            det.bbox.size_y            = float(y1 - y0)

            hyp                     = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(class_id)
            hyp.hypothesis.score    = float(score)
            det.results.append(hyp)
            det_array.detections.append(det)

        self.pub_detections.publish(det_array)

        # Log consola
        if detections:
            names = [
                f'{self.labels.get(c, str(c))} {s:.0%}'
                for c, s, *_ in detections
            ]
            self.get_logger().info('Detectado: ' + ' | '.join(names))

        # ── Imagen anotada ────────────────────────────────────────
        if self.pub_image is not None and CV2_AVAILABLE and detections:
            annotated = frame_bgr.copy()
            for class_id, score, x0, y0, x1, y1 in detections:
                color = BOX_COLORS[int(class_id) % len(BOX_COLORS)]
                label = self.labels.get(class_id, str(class_id))
                text  = f'{label} {score:.0%}'

                cv2.rectangle(annotated, (x0, y0), (x1, y1), color, 2)
                (tw, th), _ = cv2.getTextSize(
                    text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(
                    annotated,
                    (x0, y0 - th - 6), (x0 + tw + 4, y0),
                    color, -1)
                cv2.putText(
                    annotated, text, (x0 + 2, y0 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0), 1, cv2.LINE_AA)

            ann_msg        = self.bridge.cv2_to_imgmsg(
                annotated, encoding='bgr8')
            ann_msg.header = msg.header
            self.pub_image.publish(ann_msg)

    # ── Parseo de outputs ─────────────────────────────────────────

    def _parse_cpu(self, boxes, classes, scores, num_det, img_w, img_h):
        """Parsea outputs del modelo CPU postprocess."""
        results = []
        for i in range(min(num_det, self.max_detections)):
            score = float(scores[i])
            if score < self.score_threshold:
                continue
            class_id        = int(classes[i])
            ymin, xmin, ymax, xmax = boxes[i]
            x0 = int(np.clip(xmin * img_w, 0, img_w - 1))
            y0 = int(np.clip(ymin * img_h, 0, img_h - 1))
            x1 = int(np.clip(xmax * img_w, 0, img_w - 1))
            y1 = int(np.clip(ymax * img_h, 0, img_h - 1))
            results.append((class_id, score, x0, y0, x1, y1))
        return results

    def _parse_tpu(self, objects, img_w, img_h):
        """Parsea objetos de pycoral.adapters.detect."""
        results = []
        for obj in objects[:self.max_detections]:
            bbox = obj.bbox
            x_s  = img_w / self.model_w
            y_s  = img_h / self.model_h
            x0   = int(bbox.xmin * x_s)
            y0   = int(bbox.ymin * y_s)
            x1   = int(bbox.xmax * x_s)
            y1   = int(bbox.ymax * y_s)
            results.append((obj.id, float(obj.score), x0, y0, x1, y1))
        return results


# ──────────────────────────────────────────────────────────────────
# Entrypoint
# ──────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CoralDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()