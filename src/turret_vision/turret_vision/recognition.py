#!/usr/bin/env python3
"""
recognition.py
==============
Pure vision node for face detection and recognition with live CV2 window.
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
import threading


class RecognitionNode(Node):

    def __init__(self):
        super().__init__('recognition')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('target_image_path', '')
        self.declare_parameter('similarity_threshold', 0.35)
        self.declare_parameter('detection_size', 320)
        self.declare_parameter('show_window', True)

        self.target_path = self.get_parameter('target_image_path').value
        self.threshold = self.get_parameter('similarity_threshold').value
        det_size = self.get_parameter('detection_size').value
        self.show_window = self.get_parameter('show_window').value

        self.get_logger().info(f'Target path: {self.target_path}')
        self.get_logger().info(f'Threshold: {self.threshold}')
        self.get_logger().info(f'Show window: {self.show_window}')

        # ── InsightFace model ─────────────────────────────────────────────────
        self.app = None
        self.target_emb = None
        self._det_size = (det_size, det_size)
        self._load_model()

        # ── State ─────────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self._frame_count = 0
        self._last_display_time = time.time()
        self._display_interval = 0.033  # ~30 FPS for display

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        self.create_subscription(Bool, '/capture_request', self._capture_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_match = self.create_publisher(Bool, '/match_found', 10)
        self._pub_herr = self.create_publisher(Float32, '/horizontal_error', 10)
        self._pub_verr = self.create_publisher(Float32, '/vertical_error', 10)
        self._pub_sim = self.create_publisher(Float32, '/best_similarity', 10)

        # ── CV2 Window ────────────────────────────────────────────────────────
        if self.show_window:
            try:
                cv2.namedWindow('Face Recognition', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Face Recognition', 640, 480)
                cv2.moveWindow('Face Recognition', 100, 100)
                self.get_logger().info('CV2 window created successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to create CV2 window: {e}')
                self.show_window = False

        # ── Timer for continuous display ─────────────────────────────────────
        self._display_timer = self.create_timer(0.05, self._display_callback)
        
        self.get_logger().info('Recognition node ready.')

    def _load_model(self):
        try:
            from insightface.app import FaceAnalysis
        except ImportError:
            self.get_logger().error('insightface not installed!')
            return

        self.get_logger().info('Loading InsightFace buffalo_sc model...')
        try:
            self.app = FaceAnalysis(
                name='buffalo_sc',
                providers=['CPUExecutionProvider']  # Use CPU if CUDA not available
            )
            self.app.prepare(ctx_id=-1, det_size=self._det_size)  # -1 for CPU

            # Warmup
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            self.app.get(dummy)
            self.get_logger().info('Model loaded successfully')

            if self.target_path and os.path.exists(self.target_path):
                self._encode_target()
            else:
                self.get_logger().error(f'Target image not found: {self.target_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')

    def _encode_target(self):
        try:
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
            self.get_logger().info('Target face encoded successfully.')
        except Exception as e:
            self.get_logger().error(f'Error encoding target: {e}')

    def _image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._frame_lock:
                self._latest_frame = frame
                self._frame_count += 1
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')

    def _capture_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Capture request received')
            self._process_frame()

    def _display_callback(self):
        """Timer callback to continuously show the latest frame with detections"""
        if not self.show_window:
            return
            
        if self._latest_frame is None:
            return

        # Limit display rate
        current_time = time.time()
        if current_time - self._last_display_time < self._display_interval:
            return
        self._last_display_time = current_time

        # Get a copy of the latest frame
        with self._frame_lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame.copy()

        # Process and display
        self._display_frame(frame)

    def _display_frame(self, frame):
        """Display frame with face detections"""
        h, w = frame.shape[:2]
        
        # Create display frame
        display_frame = frame.copy()
        
        # Check if we have the model and target embedding
        if self.app is None or self.target_emb is None:
            cv2.putText(display_frame, 'Model not loaded', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow('Face Recognition', display_frame)
            cv2.waitKey(1)
            return

        try:
            # Detect faces
            t0 = time.time()
            faces = self.app.get(display_frame)
            dt = (time.time() - t0) * 1000

            best_sim = -1.0
            best_face = None

            # Process all detected faces
            for face in faces:
                sim = float(np.dot(self.target_emb, face.normed_embedding))
                x1, y1, x2, y2 = map(int, face.bbox)
                
                # Draw rectangle for every detected face
                color = (0, 255, 0) if sim >= self.threshold else (0, 0, 255)
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(display_frame, f'sim={sim:.3f}', (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                if sim > best_sim:
                    best_sim = sim
                    best_face = face

            # Highlight best face
            if best_face is not None:
                matched = best_sim >= self.threshold
                x1, y1, x2, y2 = map(int, best_face.bbox)
                color = (0, 255, 0) if matched else (0, 0, 255)
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 3)
                cv2.putText(display_frame, f'BEST: {best_sim:.3f}', (x1, y1-30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                if matched:
                    cv2.putText(display_frame, 'MATCH FOUND!', (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    cv2.putText(display_frame, f'Best sim: {best_sim:.3f} < {self.threshold}', (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(display_frame, 'No faces detected', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Display info
            cv2.putText(display_frame, f'Faces: {len(faces)}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display_frame, f'Threshold: {self.threshold:.2f}', (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display_frame, f'Detect: {dt:.0f}ms', (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display_frame, f'Frame: {self._frame_count}', (10, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Show window
            cv2.imshow('Face Recognition', display_frame)
            cv2.waitKey(1)  # Required for window update

        except Exception as e:
            self.get_logger().error(f'Error displaying frame: {e}')
            # Show error on frame
            cv2.putText(display_frame, f'Error: {str(e)[:50]}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow('Face Recognition', display_frame)
            cv2.waitKey(1)

    def _process_frame(self):
        """Process frame for capture request"""
        if self.app is None or self.target_emb is None:
            self.get_logger().warn('Model or target embedding not ready')
            return

        with self._frame_lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame.copy()

        h, w = frame.shape[:2]

        try:
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
                save_dir = os.path.join(os.path.expanduser('~'), 'mercury')
                os.makedirs(save_dir, exist_ok=True)
                save_path = os.path.join(save_dir, 'best_match.jpg')
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
            
        except Exception as e:
            self.get_logger().error(f'Error in _process_frame: {e}')

    def destroy_node(self):
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()