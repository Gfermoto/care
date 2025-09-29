#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CareSafety(Node):
    def __init__(self):
        super().__init__('care_safety_controller_node')
        self.declare_parameter('mode', 'mock')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        
        self.sub_targets = self.create_subscription(String, '/care/targets_raw', self.on_targets, 10)
        self.sub_status = self.create_subscription(String, '/care/system_status_raw', self.on_status, 10)
        
        self.cmd_pub = self.create_publisher(String, '/care/safety_cmd', 10)
        
        self.get_logger().info(f'Safety Node in {self.mode.upper()} mode')
        self.emergency_active = False
    
    def on_targets(self, msg: String):
        # Здесь будет логика безопасности (пока — заглушка)
        if 'dist=800' in msg.data and not self.emergency_active:
            self.emergency_active = True
            cmd = String()
            cmd.data = 'EMERGENCY_STOP'
            self.cmd_pub.publish(cmd)
            self.get_logger().warn('EMERGENCY_STOP issued by safety logic')
    
    def on_status(self, msg: String):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CareSafety()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
