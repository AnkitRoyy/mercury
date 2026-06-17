import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GpsRelay(Node):
    def __init__(self):
        super().__init__('gps_relay')
        self.sub = self.create_subscription(NavSatFix, '/gps', self.cb, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps_fixed', 10)

    def cb(self, msg):
        msg.header.frame_id = 'gps_link'
        msg.position_covariance = [1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 1.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
