import cv2
import datetime
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraRecorderNode(Node):

    def __init__(self):
        super().__init__("camera_recorder_node")

        self.bridge    = CvBridge()
        self.recorder  = None

        # ── recording path ────────────────────────────────────────────────────
        timestamp      = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        record_dir     = os.path.expanduser("~/ugv_recordings")
        os.makedirs(record_dir, exist_ok=True)
        self.record_path = os.path.join(record_dir, f"{timestamp}.avi")
        self.get_logger().info(f"Recording will save to: {self.record_path}")

        self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10)

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # ── init recorder on first frame (need frame size) ────────────────────
        if self.recorder is None:
            h, w   = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"XVID")
            self.recorder = cv2.VideoWriter(
                self.record_path, fourcc, 10.0, (w, h))
            self.get_logger().info(
                f"VideoWriter opened: {w}x{h} @ 10fps → {self.record_path}")

        # ── write frame ───────────────────────────────────────────────────────
        self.recorder.write(frame)

        # ── display raw feed ──────────────────────────────────────────────────
        cv2.imshow("Raw Camera Feed", frame)
        cv2.waitKey(1)

    def shutdown(self):
        if self.recorder is not None:
            self.recorder.release()
            self.get_logger().info(f"Recording saved: {self.record_path}")
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()