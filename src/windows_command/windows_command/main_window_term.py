import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class TerminalMonitorNode(Node):
    def __init__(self):
        super().__init__('terminal_monitor_node')
        self.get_logger().info('Terminal monitor node started.')

        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/realsense_camera/color/image_raw',
            self.image_callback,
            10)
        
        self.discovery_timer = self.create_timer(5.0, self.discovery_callback) # Check every 5 seconds

    def image_callback(self, msg):
        self.get_logger().info('Received an image frame. Topic connection is live.')

    def discovery_callback(self):
        self.get_logger().info('--- Discovery Check ---')
        
        node_names = self.get_node_names_and_namespaces()
        self.get_logger().info(f'Visible Nodes ({len(node_names)}):')
        for name, namespace in node_names:
            self.get_logger().info(f'  - {namespace}{name}')
            
        topic_names = self.get_topic_names_and_types()
        self.get_logger().info(f'Visible Topics ({len(topic_names)}):')
        for name, types in topic_names:
            self.get_logger().info(f'  - {name} ({", ".join(types)})')
        
        self.get_logger().info('-----------------------')


def main(args=None):
    rclpy.init(args=args)
    monitor_node = TerminalMonitorNode()
    
    try:
        rclpy.spin(monitor_node)
    except KeyboardInterrupt:
        pass
    finally:
        monitor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()