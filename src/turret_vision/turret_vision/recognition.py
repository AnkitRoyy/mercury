#!/usr/bin/env python3
"""
recognition.py
==============
Pure vision node for face detection and recognition.

Subscribes:
    /camera/image_raw          (sensor_msgs/Image)
    /capture_request           (std_msgs/Bool)  -- triggers one capture+match

Publishes:
    /match_found               (std_msgs/Bool)
    /horizontal_error          (std_msgs/Float32)  pixels from centre (+ = right)
    /vertical_error            (std_msgs/Float32)  pixels from centre (+ = down)
    /best_similarity           (std_msgs/Float32)

Parameters:
    target_image_path   (string)  path to target face image
    similarity_threshold (float)  default 0.35
    detection_size       (int)    InsightFace input size, default 320
"""

import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


class RecognitionNode(Node):

    def __init__(self):
        super().__init__('recognition')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('target_image_path', '')
        self.declare_parameter('similarity_threshold', 0.35)
        self.declare_parameter('detection_size', 320)

        self.target_path = self.get_parameter('target_image_path').value
        self.threshold = self.get_parameter('similarity_threshold').value
        det_size = self.get_parameter('detection_size').value

        # ── InsightFace model ─────────────────────────────────────────────────
        self.app = None
        self.target_emb = None
        self._det_size = (det_size, det_size)
        self._load_model()

        # ── State ─────────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self._latest_frame = None

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Image, '/image_raw', self._image_cb, 10)
        self.create_subscription(Bool, '/capture_request', self._capture_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_match = self.create_publisher(Bool, '/match_found', 10)
        self._pub_herr = self.create_publisher(Float32, '/horizontal_error', 10)
        self._pub_verr = self.create_publisher(Float32, '/vertical_error', 10)
        self._pub_sim = self.create_publisher(Float32, '/best_similarity', 10)

        self.get_logger().info('Recognition node ready.')

    def _load_model(self):
        try:
            from insightface.app import FaceAnalysis
        except ImportError:
            self.get_logger().error('insightface not installed!')
            return

        self.get_logger().info('Loading InsightFace buffalo_sc model...')
        self.app = FaceAnalysis(
            name='buffalo_sc',
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )
        self.app.prepare(ctx_id=0, det_size=self._det_size)

        # Warmup
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self.app.get(dummy)

        if self.target_path:
            self._encode_target()

    def _encode_target(self):
        img = cv2.imread(self.target_path)
        if img is None:
            self.get_logger().error(f'Cannot read target image: {self.target_path}')
            return
        faces = self.app.get(img)
        if not faces:
            self.get_logger().error('No face found in target image!')
            return
        largest = max(faces, key=lambda f: (f.bbox[2]-f.bbox[0])*(f.bbox[3]-f.bbox[1]))
        self.target_emb = largest.normed_embedding
        self.get_logger().info('Target face encoded.')

    def _image_cb(self, msg: Image):
        try:
            self._latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')

    def _capture_cb(self, msg: Bool):
        if msg.data:
            self._process_frame()

    def _process_frame(self):
        if self.app is None or self.target_emb is None or self._latest_frame is None:
            return

        frame = self._latest_frame.copy()
        h, w = frame.shape[:2]

        t0 = time.time()
        faces = self.app.get(frame)
        dt = (time.time() - t0) * 1000

        if not faces:
            self._pub_match.publish(Bool(data=False))
            self._pub_sim.publish(Float32(data=0.0))
            return

        best_sim = -1.0
        best_face = None
        for face in faces:
            sim = float(np.dot(self.target_emb, face.normed_embedding))
            if sim > best_sim:
                best_sim = sim
                best_face = face

        matched = best_sim >= self.threshold

        if matched:
            save_path = os.path.join(os.path.expanduser('~'), 'mercury', 'best_match.jpg') #change
            x1, y1, x2, y2 = map(int, best_face.bbox)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'sim={best_sim:.3f}', (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imwrite(save_path, frame)
            self.get_logger().info(f'Match saved: {save_path}')

        cx = (best_face.bbox[0] + best_face.bbox[2]) / 2.0
        cy = (best_face.bbox[1] + best_face.bbox[3]) / 2.0
        h_err = cx - (w / 2.0)
        v_err = cy - (h / 2.0)

        self.get_logger().debug(f'[{dt:.0f}ms] sim={best_sim:.3f} matched={matched}')
        self._pub_match.publish(Bool(data=matched))
        self._pub_herr.publish(Float32(data=float(h_err)))
        self._pub_verr.publish(Float32(data=float(v_err)))
        self._pub_sim.publish(Float32(data=float(best_sim)))


def main(args=None):
    rclpy.init(args=args)
    node = RecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()