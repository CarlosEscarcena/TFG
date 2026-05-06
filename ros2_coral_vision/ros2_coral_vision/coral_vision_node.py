#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from cv_bridge import CvBridge
import cv2
import numpy as np
import time

try:
    from pycoral.utils.edgetpu import make_interpreter
    from pycoral.adapters import common, detect, classify
    from pycoral.utils.dataset import read_label_file
    CORAL_AVAILABLE = True
except ImportError:
    CORAL_AVAILABLE = False


class CoralVisionNode(Node):
    def __init__(self):
        super().__init__('coral_vision_node')

        # ── Parámetros ──────────────────────────────────────────────────────────
        self.declare_parameter('model_path', '')
        self.declare_parameter('labels_path', '')
        self.declare_parameter('task', 'detection')
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/coral/image_detections')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('use_edgetpu', True)

        model_path     = self.get_parameter('model_path').value
        labels_path    = self.get_parameter('labels_path').value
        self.task      = self.get_parameter('task').value
        self.threshold = self.get_parameter('score_threshold').value
        input_topic    = self.get_parameter('input_topic').value
        output_topic   = self.get_parameter('output_topic').value
        self.pub_annot = self.get_parameter('publish_annotated').value
        use_edgetpu    = self.get_parameter('use_edgetpu').value

        # ── Verificar Coral ──────────────────────────────────────────────────────
        if not CORAL_AVAILABLE:
            self.get_logger().error(
                'pycoral no está instalado. Instálalo con:\n'
                '  pip install pycoral ai-edge-litert'
            )
            raise RuntimeError('pycoral no disponible')

        import ctypes, os
        for _lib in ['libedgetpu.so.1', 'libedgetpu.so']:
            for _dir in ['/usr/lib/aarch64-linux-gnu', '/usr/lib/x86_64-linux-gnu',
                         '/usr/local/lib', '/usr/lib']:
                _path = os.path.join(_dir, _lib)
                if os.path.exists(_path):
                    try:
                        ctypes.CDLL(_path)
                        self.get_logger().info(f'libedgetpu cargada desde {_path}')
                    except Exception:
                        pass
                    break

        if not model_path:
            self.get_logger().error('Parámetro "model_path" vacío.')
            raise RuntimeError('model_path no especificado')

        # ── Cargar modelo ────────────────────────────────────────────────────────
        self.get_logger().info(f'Cargando modelo: {model_path}')
        try:
            if use_edgetpu:
                import ctypes, os
                _found = False
                for _lib in ['libedgetpu.so.1.0', 'libedgetpu.so.1', 'libedgetpu.so']:
                    for _dir in ['/usr/lib/aarch64-linux-gnu', '/usr/lib/x86_64-linux-gnu',
                                 '/usr/local/lib', '/usr/lib']:
                        _p = os.path.join(_dir, _lib)
                        if os.path.exists(_p):
                            try:
                                ctypes.CDLL(_p)
                                self.get_logger().info(f'libedgetpu encontrada: {_p}')
                                _found = True
                            except OSError as _e:
                                self.get_logger().warn(f'No se pudo cargar {_p}: {_e}')
                        if _found:
                            break
                    if _found:
                        break
                self.interpreter = make_interpreter(model_path)
            else:
                import tflite_runtime.interpreter as tflite
                self.interpreter = tflite.Interpreter(model_path=model_path)
            self.interpreter.allocate_tensors()
        except Exception as e:
            self.get_logger().error(f'Error al cargar el modelo: {e}')
            raise

        # ── Etiquetas ────────────────────────────────────────────────────────────
        self.labels = {}
        if labels_path:
            try:
                self.labels = read_label_file(labels_path)
                self.get_logger().info(f'Etiquetas cargadas: {len(self.labels)} clases')
            except Exception as e:
                self.get_logger().warn(f'No se pudieron cargar las etiquetas: {e}')

        # ── Dimensiones de entrada del modelo ────────────────────────────────────
        input_details = self.interpreter.get_input_details()
        _, self.input_h, self.input_w, _ = input_details[0]['shape']
        self.get_logger().info(
            f'Modelo listo | tarea={self.task} | '
            f'entrada={self.input_w}x{self.input_h} | '
            f'threshold={self.threshold}'
        )

        # ── CV Bridge ────────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── QoS ─────────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Suscriptor ───────────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            Image, input_topic, self.image_callback, qos)

        # ── Publicadores ─────────────────────────────────────────────────────────
        if self.task == 'detection':
            self.pub_det = self.create_publisher(Detection2DArray, output_topic, 10)
        else:
            self.pub_cls = self.create_publisher(String, output_topic, 10)

        if self.pub_annot:
            self.pub_img = self.create_publisher(Image, '/coral/image_annotated', 10)

        # ── Stats ────────────────────────────────────────────────────────────────
        self._frame_count = 0
        self._total_ms = 0.0
        # Últimas detecciones para mostrarlas en el log periódico
        self._last_detections = []
        self.create_timer(10.0, self._log_stats)

        self.get_logger().info('✓ CoralVisionNode iniciado')

    # ────────────────────────────────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        h_orig, w_orig = cv_image.shape[:2]

        # Redimensionar para el modelo
        resized = cv2.resize(cv_image, (self.input_w, self.input_h))

        t0 = time.perf_counter()
        common.set_input(self.interpreter, resized)
        self.interpreter.invoke()
        elapsed_ms = (time.perf_counter() - t0) * 1000

        self._frame_count += 1
        self._total_ms += elapsed_ms

        if self.task == 'detection':
            self._handle_detection(msg, cv_image, w_orig, h_orig, elapsed_ms)
        else:
            self._handle_classification(msg, elapsed_ms)

    # ── Detección de objetos ──────────────────────────────────────────────────
    def _handle_detection(self, msg, cv_image, w_orig, h_orig, elapsed_ms):
        # IMPORTANTE: scale=(1,1) porque queremos las coords en espacio del modelo
        # y luego escalamos manualmente a la imagen original
        objs = detect.get_objects(self.interpreter, self.threshold, (1.0, 1.0))

        # Factor de escala del modelo a la imagen original
        scale_x = w_orig / self.input_w
        scale_y = h_orig / self.input_h

        det_array = Detection2DArray()
        det_array.header = msg.header
        self._last_detections = []

        for obj in objs:
            # Escalar bbox al tamaño real de la imagen
            xmin = int(obj.bbox.xmin * scale_x)
            ymin = int(obj.bbox.ymin * scale_y)
            xmax = int(obj.bbox.xmax * scale_x)
            ymax = int(obj.bbox.ymax * scale_y)

            label = self.labels.get(obj.id, f'id_{obj.id}')
            score = float(obj.score)

            # Guardar para el log
            self._last_detections.append((label, score))

            # Publicar por terminal cada detección
            self.get_logger().info(
                f'  🔍 {label:<20} {score*100:5.1f}%'
            )

            # Mensaje ROS2
            d = Detection2D()
            d.header = msg.header
            d.bbox.center.position.x = float((xmin + xmax) / 2)
            d.bbox.center.position.y = float((ymin + ymax) / 2)
            d.bbox.size_x = float(xmax - xmin)
            d.bbox.size_y = float(ymax - ymin)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(obj.id)
            hyp.hypothesis.score = score
            d.results.append(hyp)
            det_array.detections.append(d)

        self.pub_det.publish(det_array)

        # Imagen anotada con bboxes escalados correctamente
        if self.pub_annot:
            annotated = self._draw_detections(cv_image.copy(), objs, scale_x, scale_y)
            img_msg = self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR), encoding='bgr8')
            img_msg.header = msg.header
            self.pub_img.publish(img_msg)

    # ── Clasificación ─────────────────────────────────────────────────────────
    def _handle_classification(self, msg, elapsed_ms):
        classes = classify.get_classes(self.interpreter, top_k=5, score_threshold=self.threshold)

        results = []
        for c in classes:
            label = self.labels.get(c.id, f'id_{c.id}')
            results.append(f'{label}: {c.score:.3f}')
            self.get_logger().info(f'  🏷  {label:<20} {c.score*100:5.1f}%')

        out_msg = String()
        out_msg.data = ' | '.join(results) if results else 'sin_deteccion'
        self.pub_cls.publish(out_msg)

    # ── Dibujado de bounding boxes (coords ya escaladas) ─────────────────────
    def _draw_detections(self, image, objs, scale_x, scale_y):
        for obj in objs:
            # Aplicar escala del modelo → imagen original
            xmin = int(obj.bbox.xmin * scale_x)
            ymin = int(obj.bbox.ymin * scale_y)
            xmax = int(obj.bbox.xmax * scale_x)
            ymax = int(obj.bbox.ymax * scale_y)

            label = self.labels.get(obj.id, f'id_{obj.id}')
            score = obj.score
            color = (0, 255, 0)

            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 2)

            text = f'{label} {score:.0%}'
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            # Fondo del texto (asegurar que no se salga por arriba)
            ty = max(ymin, th + 6)
            cv2.rectangle(image,
                          (xmin, ty - th - 6),
                          (xmin + tw + 4, ty),
                          color, -1)
            cv2.putText(image, text,
                        (xmin + 2, ty - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        return image

    # ── Estadísticas periódicas ───────────────────────────────────────────────
    def _log_stats(self):
        if self._frame_count > 0:
            avg = self._total_ms / self._frame_count
            fps = 1000.0 / avg if avg > 0 else 0
            self.get_logger().info(
                f'── Stats ── frames={self._frame_count} | '
                f'latencia={avg:.1f} ms | FPS≈{fps:.1f}'
            )


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CoralVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f'[ERROR] {e}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()