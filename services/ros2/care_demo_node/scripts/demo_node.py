#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from care_common.srv import ConfigController
from care_common.msg import RadarTargets as MsgRadarTargets
from care_common.msg import SystemStatus as MsgSystemStatus
import json
import time

try:
    import can  # python-can
except Exception:
    can = None

KV_MAP_CONTROLLER = {
    'bitrate': (0x10, {
        125000: 0x00,
        250000: 0x01,
        500000: 0x02,
        1000000: 0x03,
    }),
    'mode': (0x11, {
        'normal': 0x00,
        'silent': 0x01,
    }),
}

KV_MAP_RADAR = {
    'max_targets': (0x20, None),
    'range_m': (0x21, None),
    'update_hz': (0x22, None),
    'sensitivity': (0x23, {'low': 0, 'med': 1, 'high': 2}),
}

class CareDemo(Node):
    def __init__(self):
        super().__init__('care_demo_node')
        self.declare_parameter('mode', 'mock')
        self.declare_parameter('can_interface', 'can0')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        
        # raw подписки (на переходный период)
        self.sub_targets_raw = self.create_subscription(String, '/care/targets_raw', self.on_targets_raw, 10)
        self.sub_status_raw = self.create_subscription(String, '/care/system_status_raw', self.on_status_raw, 10)
        
        # typed подписки
        self.sub_targets = self.create_subscription(MsgRadarTargets, '/care/targets', self.on_targets_typed, 10)
        self.sub_status = self.create_subscription(MsgSystemStatus, '/care/system_status', self.on_status_typed, 10)
        
        self.markers_pub = self.create_publisher(MarkerArray, '/care/rviz/targets', 10)
        self.status_pub = self.create_publisher(Marker, '/care/rviz/status', 10)
        
        # Сервис конфигурации (контроллер/радар)
        self.config_srv = self.create_service(ConfigController, '/care/configure_controller', self.on_config_request)
        
        # Подписка на ACK от моста
        self.ack_last = None
        self.sub_ack = self.create_subscription(String, '/care/config_ack_raw', self.on_ack, 10)
        
        # Инициализация CAN
        self.can_bus = None
        if can is not None:
            try:
                self.can_bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
                self.get_logger().info(f'CAN ready on {self.can_interface} for config commands (0x400/0x401)')
            except Exception as e:
                self.get_logger().warn(f'CAN not available on {self.can_interface}: {e}')
        else:
            self.get_logger().warn('python-can не установлен. Конфигурация через CAN недоступна.')
        
        self.seq = 0
        self.get_logger().info(f'DEMO started in {self.mode.upper()} mode')
    
    def on_ack(self, msg: String):
        self.ack_last = (self.get_clock().now(), msg.data)
    
    # ===== Typed визуализация =====
    def on_targets_typed(self, msg: MsgRadarTargets):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        idx = 0
        for t in msg.targets:
            if not t.valid:
                continue
            m = Marker()
            m.header.frame_id = 'care_radar'
            m.header.stamp = now
            m.ns = 'targets'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(t.x) / 1000.0
            m.pose.position.y = float(t.y) / 1000.0
            m.pose.position.z = 0.0
            m.scale.x = 0.08
            m.scale.y = 0.08
            m.scale.z = 0.08
            m.color.r = 0.2
            m.color.g = 0.8
            m.color.b = 1.0
            m.color.a = 0.9
            markers.markers.append(m)
            idx += 1
        if markers.markers:
            self.markers_pub.publish(markers)
    
    def on_status_typed(self, msg: MsgSystemStatus):
        m = Marker()
        m.header.frame_id = 'care_radar'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'status'
        m.id = 999
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = -0.5
        m.pose.position.z = 0.1
        m.scale.z = 0.15
        if msg.emergency_stop_active:
            m.text = f'ACTIVE={msg.active_devices} EMERGENCY=YES'
            m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2
        else:
            m.text = f'ACTIVE={msg.active_devices} EMERGENCY=NO'
            m.color.r = 0.2; m.color.g = 1.0; m.color.b = 0.2
        m.color.a = 1.0
        self.status_pub.publish(m)
    
    # ===== Raw визуализация (fallback) =====
    def on_targets_raw(self, msg: String):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        idx = 0
        for part in msg.data.split(';'):
            part = part.strip()
            if not part:
                continue
            try:
                fields = {kv.split('=')[0]: kv.split('=')[1] for kv in part.split() if '=' in kv}
                x_mm = int(fields.get('x', '0'))
                y_mm = int(fields.get('y', '0'))
                m = Marker()
                m.header.frame_id = 'care_radar'
                m.header.stamp = now
                m.ns = 'targets'
                m.id = idx
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = x_mm / 1000.0
                m.pose.position.y = y_mm / 1000.0
                m.pose.position.z = 0.0
                m.scale.x = 0.08
                m.scale.y = 0.08
                m.scale.z = 0.08
                m.color.r = 0.2
                m.color.g = 0.8
                m.color.b = 1.0
                m.color.a = 0.9
                markers.markers.append(m)
                idx += 1
            except Exception:
                continue
        if markers.markers:
            self.markers_pub.publish(markers)
    
    def on_status_raw(self, msg: String):
        try:
            fields = {kv.split('=')[0]: kv.split('=')[1] for kv in msg.data.split() if '=' in kv}
            active = int(fields.get('active', '0'))
            emergency = int(fields.get('emergency', '0'))
            m = Marker()
            m.header.frame_id = 'care_radar'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'status'
            m.id = 999
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = 0.0
            m.pose.position.y = -0.5
            m.pose.position.z = 0.1
            m.scale.z = 0.15
            if emergency:
                m.text = f'ACTIVE={active} EMERGENCY=YES'
                m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2
            else:
                m.text = f'ACTIVE={active} EMERGENCY=NO'
                m.color.r = 0.2; m.color.g = 1.0; m.color.b = 0.2
            m.color.a = 1.0
            self.status_pub.publish(m)
        except Exception:
            pass
    
    # ===== Конфигурация CAN (без изменений) =====
    def on_config_request(self, request: ConfigController.Request, response: ConfigController.Response):
        section = request.config_section.strip().lower()
        try:
            cfg = json.loads(request.config_data) if request.config_data else {}
        except Exception as e:
            response.success = False
            response.message = f'Invalid JSON: {e}'
            return response
        
        if self.can_bus is None:
            response.success = False
            response.message = 'CAN bus not available'
            return response
        
        # Выбор базы ID
        base_id = 0x400 if section in ('can','controller') else 0x401 if section == 'radar' else None
        if base_id is None:
            response.success = False
            response.message = f'Unknown config_section: {section}'
            return response
        
        # Команда (SET/GET/RESET)
        cmd = cfg.pop('__cmd__', 'set').lower()  # специальное поле для выбора команды
        if cmd == 'set':
            success, msg = self._send_can_config(base_id, cfg, KV_MAP_CONTROLLER if base_id==0x400 else KV_MAP_RADAR)
        elif cmd == 'get':
            success, msg = self._send_can_simple(base_id, 0x02)
        elif cmd == 'reset':
            success, msg = self._send_can_simple(base_id, 0x04)
        else:
            response.success = False
            response.message = f'Unknown __cmd__: {cmd}'
            return response
        
        # SAVE
        if success and request.save_to_flash and cmd == 'set':
            try:
                self._send_can_simple(base_id, 0x03)
            except Exception as e:
                self.get_logger().warn(f'SAVE failed: {e}')
        
        # Ожидание ACK (до 500мс)
        ack_ok = self._wait_for_ack(timeout_ms=500)
        if not ack_ok:
            self.get_logger().warn('ACK not received')
        
        response.success = success and ack_ok
        response.message = (msg + (' (ACK OK)' if ack_ok else ' (no ACK)')) if success else msg
        response.applied_config = json.dumps(cfg)
        response.applied_time = self.get_clock().now().to_msg()
        response.device_restart_required = (cmd == 'reset')
        return response
    
    def _wait_for_ack(self, timeout_ms: int = 500) -> bool:
        start = self.get_clock().now().nanoseconds // 1_000_000
        while (self.get_clock().now().nanoseconds // 1_000_000) - start < timeout_ms:
            if self.ack_last is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False
    
    def _send_can_simple(self, base_id: int, cmd: int):
        payload = [0]*8
        payload[0] = cmd & 0xFF
        try:
            msg = can.Message(arbitration_id=base_id, data=bytes(payload), is_extended_id=False)
            self.can_bus.send(msg)
            self.get_logger().info(f'Sent CAN cmd 0x{base_id:03X} CMD=0x{cmd:02X}')
            return True, 'Command sent via CAN'
        except Exception as e:
            return False, f'CAN send failed: {e}'
    
    def _send_can_config(self, base_id: int, cfg: dict, kv_map: dict):
        CMD_SET = 0x01
        payload = [0]*8
        payload[0] = CMD_SET
        kv_pairs = []
        for key, val in list(cfg.items()):
            if key not in kv_map:
                continue
            code, lut = kv_map[key]
            if lut is None:
                try:
                    v = int(val)
                except Exception:
                    continue
            else:
                try:
                    v = lut[val] if isinstance(val, str) else int(val)
                except Exception:
                    continue
            kv_pairs.append((code, v & 0xFF))
            if len(kv_pairs) >= 3:
                break
        idx = 1
        for (k, v) in kv_pairs:
            payload[idx] = k; payload[idx+1] = v
            idx += 2
        try:
            msg = can.Message(arbitration_id=base_id, data=bytes(payload), is_extended_id=False)
            self.can_bus.send(msg)
            self.get_logger().info(f'Sent CAN config 0x{base_id:03X}: {payload}')
            return True, 'Config sent via CAN'
        except Exception as e:
            return False, f'CAN send failed: {e}'

def main(args=None):
    rclpy.init(args=args)
    node = CareDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

