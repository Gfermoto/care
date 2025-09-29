#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

try:
    import can  # python-can
except Exception:
    can = None

from care_common.msg import RadarTarget as MsgRadarTarget
from care_common.msg import RadarTargets as MsgRadarTargets
from care_common.msg import SystemStatus as MsgSystemStatus
from std_msgs.msg import Header

class CareCanBridge(Node):
    def __init__(self):
        super().__init__('care_can_bridge_node')
        self.declare_parameter('mode', 'real')  # mock | real
        self.declare_parameter('can_interface', 'can0')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        
        # RAW publishers (сохранение обратной совместимости)
        self.targets_pub = self.create_publisher(String, '/care/targets_raw', 10)
        self.status_pub = self.create_publisher(String, '/care/system_status_raw', 10)
        self.emergency_pub = self.create_publisher(String, '/care/emergency_raw', 10)
        self.config_ack_pub = self.create_publisher(String, '/care/config_ack_raw', 10)
        
        # Typed publishers
        self.targets_typed_pub = self.create_publisher(MsgRadarTargets, '/care/targets', 10)
        self.status_typed_pub = self.create_publisher(MsgSystemStatus, '/care/system_status', 10)
        
        if self.mode == 'mock':
            self.get_logger().info('Running in MOCK mode')
            self.timer = self.create_timer(0.05, self.publish_mock)
        else:
            self.get_logger().info(f'Running in REAL mode via socketcan on {self.can_interface}')
            if can is None:
                self.get_logger().error('python-can не установлен. Установите пакет python-can.')
                self.timer = self.create_timer(1.0, self.publish_placeholder)
            else:
                self._can_bus = None
                self._can_thread = None
                self._can_stop = threading.Event()
                self._start_can()
    
    def _start_can(self):
        try:
            self._can_bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
            self._can_thread = threading.Thread(target=self._can_loop, daemon=True)
            self._can_thread.start()
            self.get_logger().info('CAN reader thread started')
        except Exception as e:
            self.get_logger().error(f'Не удалось открыть {self.can_interface}: {e}')
            self.timer = self.create_timer(1.0, self.publish_placeholder)
    
    def _can_loop(self):
        while not self._can_stop.is_set():
            try:
                msg = self._can_bus.recv(0.1)
                if msg is None:
                    continue
                can_id = msg.arbitration_id
                data = bytes(msg.data)
                now = self.get_clock().now().to_msg()
                # 0x200-0x202 Target Data
                if 0x200 <= can_id <= 0x202 and len(data) >= 8:
                    tid = can_id - 0x200
                    target_id = (data[0] << 8) | data[1]
                    x = (data[2] << 8) | data[3]
                    if x & 0x8000: x -= 0x10000
                    y = (data[4] << 8) | data[5]
                    if y & 0x8000: y -= 0x10000
                    dist = (data[6] << 8) | data[7]
                    # RAW
                    out_raw = String()
                    out_raw.data = f'ID=0x{can_id:03X} T{tid} id={target_id} x={x} y={y} dist={dist}'
                    self.targets_pub.publish(out_raw)
                    # TYPED: Буферизуем по устройству 1 (можно расширить)
                    targets_msg = MsgRadarTargets()
                    targets_msg.header = Header(stamp=now, frame_id='care_radar')
                    targets_msg.device_id = 1
                    t = MsgRadarTarget()
                    t.id = target_id
                    t.device_id = 1
                    t.x = x
                    t.y = y
                    t.distance = dist
                    t.angle = 0.0
                    t.speed = 0
                    t.valid = True
                    t.timestamp = now
                    t.last_seen = now
                    t.confidence = 1.0
                    t.tracking_age = 0
                    t.in_safety_zone = False
                    t.safety_distance = 0.0
                    targets_msg.targets = [t]
                    targets_msg.target_count = 1
                    targets_msg.device_status = 0
                    targets_msg.radar_status = 0
                    targets_msg.update_rate = 20.0
                    targets_msg.targets_in_safety_zone = 0
                    targets_msg.emergency_detected = False
                    targets_msg.closest_distance = float(dist)
                    self.targets_typed_pub.publish(targets_msg)
                # 0x300 System Status
                elif can_id == 0x300 and len(data) >= 2:
                    active = data[0]
                    emerg = data[1]
                    # RAW
                    out_raw = String()
                    out_raw.data = f'ID=0x300 active={active} emergency={emerg}'
                    self.status_pub.publish(out_raw)
                    # TYPED (минимальная сборка)
                    status = MsgSystemStatus()
                    status.header = Header(stamp=now, frame_id='care_radar')
                    status.overall_status = MsgSystemStatus.SYSTEM_EMERGENCY if emerg else MsgSystemStatus.SYSTEM_OK
                    status.status_message = 'Emergency stop active' if emerg else 'System OK'
                    status.active_devices = 1
                    status.offline_devices = 0
                    status.safety_system_active = True
                    status.emergency_stop_active = bool(emerg)
                    status.last_safety_check = now
                    status.system_uptime = 0.0
                    status.message_rate = 0.0
                    status.total_targets_seen = 0
                    status.emergency_events = 1 if emerg else 0
                    status.cpu_usage = 0.0
                    status.memory_usage = 0.0
                    status.can_message_count = 0
                    self.status_typed_pub.publish(status)
                # 0x100 Emergency Stop
                elif can_id == 0x100 and len(data) >= 1:
                    cmd = data[0]
                    out = String()
                    out.data = f'ID=0x100 cmd={cmd}'
                    self.emergency_pub.publish(out)
                # 0x480/0x481 ACK/ERROR от контроллера/радара
                elif can_id in (0x480, 0x481) and len(data) >= 2:
                    code = data[0]
                    detail = data[1]
                    out = String()
                    who = 'CTRL' if can_id == 0x480 else 'RADAR'
                    out.data = f'ID=0x{can_id:03X} {who} ACK code={code} detail={detail}'
                    self.config_ack_pub.publish(out)
            except Exception as e:
                self.get_logger().warn(f'CAN read error: {e}')
    
    def destroy_node(self):
        try:
            if hasattr(self, '_can_stop'):
                self._can_stop.set()
            if hasattr(self, '_can_thread') and self._can_thread:
                self._can_thread.join(timeout=1.0)
            if hasattr(self, '_can_bus') and self._can_bus:
                try:
                    self._can_bus.shutdown()
                except Exception:
                    pass
        finally:
            super().destroy_node()
    
    def publish_mock(self):
        # RAW
        msg_t = String()
        msg_t.data = 'ID=0x200 T0 id=1 x=150 y=200 dist=1200; ID=0x201 T1 id=2 x=300 y=-100 dist=2500'
        self.targets_pub.publish(msg_t)
        msg_s = String()
        msg_s.data = 'ID=0x300 active=2 emergency=0'
        self.status_pub.publish(msg_s)
        # TYPED (минимальная генерация)
        now = self.get_clock().now().to_msg()
        targets_msg = MsgRadarTargets()
        targets_msg.header = Header(stamp=now, frame_id='care_radar')
        targets_msg.device_id = 1
        t = MsgRadarTarget()
        t.id = 1; t.device_id = 1; t.x = 150; t.y = 200; t.distance = 1200; t.valid = True
        t.timestamp = now; t.last_seen = now; t.confidence = 1.0
        targets_msg.targets = [t]
        targets_msg.target_count = 1
        self.targets_typed_pub.publish(targets_msg)
        status = MsgSystemStatus()
        status.header = Header(stamp=now, frame_id='care_radar')
        status.overall_status = MsgSystemStatus.SYSTEM_OK
        status.status_message = 'System OK'
        self.status_typed_pub.publish(status)
    
    def publish_placeholder(self):
        msg = String()
        msg.data = 'waiting for CAN interface (python-can not available)'
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CareCanBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
