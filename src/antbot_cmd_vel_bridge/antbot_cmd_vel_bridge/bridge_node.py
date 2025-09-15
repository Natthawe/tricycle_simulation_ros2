#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Twist, TwistStamped


TWIST_TYPE = 'geometry_msgs/msg/Twist'
STAMPED_TYPE = 'geometry_msgs/msg/TwistStamped'


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # -------- Parameters --------
        # self.declare_parameter('use_sim_time', True)

        # INPUT: ชื่อเดียว แต่จะเลือกชนิดตาม publisher ที่เจอ (ห้ามสมัครสองชนิดพร้อมกัน)
        self.declare_parameter('input_topic', '/cmd_vel')

        # OUTPUT: ปล่อยได้สองชนิด (ต้องเป็นคนละ topic กัน)
        self.declare_parameter('publish_twist', True)
        self.declare_parameter('publish_stamped', True)
        self.declare_parameter('output_topic_twist', '/cmd_vel_out')
        self.declare_parameter('output_topic_stamped', '/tricycle_controller/cmd_vel')
        self.declare_parameter('default_frame_id', '')

        # Logging / polling
        self.declare_parameter('print_messages', True)
        self.declare_parameter('conn_log_period', 0.5)     # วินาที พิมพ์เมื่อมี sub/pub connect/disconnect
        self.declare_parameter('detect_period', 0.2)       # วินาที ตรวจชนิดอินพุต (Twist/Stamped) อัตโนมัติ

        # -------- Read params --------
        self._input_topic = str(self.get_parameter('input_topic').value)
        self._pub_twist_enabled = bool(self.get_parameter('publish_twist').value)
        self._pub_stamped_enabled = bool(self.get_parameter('publish_stamped').value)
        self._out_twist_topic = str(self.get_parameter('output_topic_twist').value)
        self._out_stamped_topic = str(self.get_parameter('output_topic_stamped').value)
        self._default_frame_id = str(self.get_parameter('default_frame_id').value)
        self._print_messages = bool(self.get_parameter('print_messages').value)
        self._conn_period = float(self.get_parameter('conn_log_period').value)
        self._detect_period = float(self.get_parameter('detect_period').value)

        qos = QoSProfile(depth=10)

        # -------- Output publishers --------
        # ป้องกันชนชนิด: ถ้าตั้งชื่อ output สองอันซ้ำกัน จะเปลี่ยนชื่อของ Twist อัตโนมัติ
        if self._pub_twist_enabled and self._pub_stamped_enabled and \
           (self._out_twist_topic == self._out_stamped_topic):
            new_twist = self._out_twist_topic + '_twist'
            self.get_logger().warn(
                f'output_topic_twist ซ้ำกับ output_topic_stamped ("{self._out_twist_topic}") '
                f'เปลี่ยน output_topic_twist เป็น "{new_twist}" อัตโนมัติ'
            )
            self._out_twist_topic = new_twist

        self.pub_twist = self.create_publisher(Twist, self._out_twist_topic, qos) \
            if self._pub_twist_enabled else None
        self.pub_stamped = self.create_publisher(TwistStamped, self._out_stamped_topic, qos) \
            if self._pub_stamped_enabled else None

        # -------- Input subscriber (dynamic) --------
        self.sub_handle = None           # ตัวจับ subscription ปัจจุบัน
        self._sub_current_type = None    # 'twist' หรือ 'stamped'
        self._last_seen_types = {'twist': 0, 'stamped': 0}  # จำนวน publisher ต่อชนิดบน input_topic

        # -------- Startup logs --------
        out_modes = []
        if self.pub_twist: out_modes.append(f'Twist->{self._out_twist_topic}')
        if self.pub_stamped: out_modes.append(f'TwistStamped->{self._out_stamped_topic}')
        self.get_logger().info(f'INPUT  : {self._input_topic} (auto-detect type)')
        self.get_logger().info(f'OUTPUT : {", ".join(out_modes) if out_modes else "none"}')

        # -------- Timers --------
        # ตรวจชนิดบน input_topic แล้วสร้าง/สลับ subscriber ตามนั้น
        self._detect_timer = self.create_timer(self._detect_period, self._ensure_subscription)

        # log การเชื่อมต่อเข้า/ออก
        self._last_out_counts = {
            'twist': self.pub_twist.get_subscription_count() if self.pub_twist else 0,
            'stamped': self.pub_stamped.get_subscription_count() if self.pub_stamped else 0,
        }
        if self._conn_period > 0.0:
            self._conn_timer = self.create_timer(self._conn_period, self._check_connections)

    # ---------- Time helper ----------
    def _now(self) -> RosTime:
        return self.get_clock().now().to_msg()

    # ---------- Pretty logs ----------
    def _print_in(self, kind: str, vx: float, wz: float, stamp: RosTime):
        if self._print_messages:
            self.get_logger().info(
                f'[IN {kind}] v={vx:.3f} m/s  w={wz:.3f} rad/s  '
                f'stamp=[{stamp.sec}.{stamp.nanosec:09d}]'
            )

    def _print_out(self, kind: str, topic: str, vx: float, wz: float, stamp: RosTime = None):
        if self._print_messages:
            if stamp is not None:
                self.get_logger().info(
                    f'[OUT {kind} -> {topic}] v={vx:.3f}  w={wz:.3f}  '
                    f'stamp=[{stamp.sec}.{stamp.nanosec:09d}]'
                )
            else:
                self.get_logger().info(
                    f'[OUT {kind} -> {topic}] v={vx:.3f}  w={wz:.3f}'
                )

    # ---------- Output helpers ----------
    def _publish_twist(self, vx: float, wz: float):
        if not self.pub_twist:
            return
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub_twist.publish(msg)
        # self._print_out('Twist', self._out_twist_topic, vx, wz)

    def _publish_stamped(self, vx: float, wz: float, stamp: RosTime, frame_id: str = ''):
        if not self.pub_stamped:
            return
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id if frame_id else ''
        msg.twist.linear.x = float(vx)
        msg.twist.angular.z = float(wz)
        self.pub_stamped.publish(msg)
        # self._print_out('TwistStamped', self._out_stamped_topic, vx, wz, stamp)

    # ---------- Dynamic subscription management ----------
    def _ensure_subscription(self):
        # ดูว่ามี publisher อะไรบ้างบน input_topic ตอนนี้
        infos = self.get_publishers_info_by_topic(self._input_topic)
        counts = {'twist': 0, 'stamped': 0}
        for info in infos:
            if info.topic_type == TWIST_TYPE:
                counts['twist'] += 1
            elif info.topic_type == STAMPED_TYPE:
                counts['stamped'] += 1

        # log เมื่อจำนวน publisher ต่อชนิดเปลี่ยน
        for k in ('twist', 'stamped'):
            if counts[k] != self._last_seen_types[k]:
                if counts[k] > self._last_seen_types[k]:
                    self.get_logger().info(
                        f'[INPUT {k}] publisher appeared on {self._input_topic} '
                        # f'({self._last_seen_types[k]} -> {counts[k]})'
                        f'({counts[k]})'
                    )
                else:
                    self.get_logger().info(
                        f'[INPUT {k}] publisher disappeared from {self._input_topic} '
                        # f'({self._last_seen_types[k]} -> {counts[k]})'
                        f'({counts[k]})'
                    )
                self._last_seen_types[k] = counts[k]

        # เลือกชนิดที่จะ subscribe: ถ้ามีทั้งสอง (ไม่น่าเกิด) ให้เลือกที่จำนวนมากกว่า
        chosen = None
        if counts['twist'] == 0 and counts['stamped'] == 0:
            # ไม่มี publisher -> ยังไม่สร้าง sub
            return
        elif counts['twist'] >= counts['stamped']:
            chosen = 'twist'
        else:
            chosen = 'stamped'

        if chosen == self._sub_current_type:
            return  # ใช้งานชนิดเดิมอยู่แล้ว

        # เปลี่ยนชนิด: destroy ของเดิมแล้วสร้างใหม่
        if self.sub_handle is not None:
            self.get_logger().info(f'Unsubscribing from {self._input_topic} ({self._sub_current_type})')
            self.destroy_subscription(self.sub_handle)
            self.sub_handle = None
            self._sub_current_type = None

        qos = QoSProfile(depth=10)
        if chosen == 'twist':
            self.sub_handle = self.create_subscription(Twist, self._input_topic, self.cb_twist, qos)
        else:
            self.sub_handle = self.create_subscription(TwistStamped, self._input_topic, self.cb_stamped, qos)

        self._sub_current_type = chosen
        self.get_logger().info(f'Subscribed to {self._input_topic} as {chosen.upper()}')

    # ---------- Callbacks ----------
    def cb_twist(self, msg: Twist):
        stamp = self._now()
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        # self._print_in('Twist', vx, wz, stamp)
        self._publish_twist(vx, wz)
        self._publish_stamped(vx, wz, stamp, self._default_frame_id)

    def cb_stamped(self, msg: TwistStamped):
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            if not hasattr(self, '_warned_zero'):
                self.get_logger().warn(
                    'Received TwistStamped with zero timestamp; replacing with now() (shown once)'
                )
                self._warned_zero = True
            stamp = self._now()
        vx = float(msg.twist.linear.x)
        wz = float(msg.twist.angular.z)
        # self._print_in('TwistStamped', vx, wz, stamp)
        self._publish_twist(vx, wz)
        frame_id = msg.header.frame_id if msg.header.frame_id else self._default_frame_id
        self._publish_stamped(vx, wz, stamp, frame_id)

    # ---------- Connection polling for outputs ----------
    def _check_connections(self):
        if self.pub_twist:
            c = self.pub_twist.get_subscription_count()
            if c != self._last_out_counts['twist']:
                if c > self._last_out_counts['twist']:
                    self.get_logger().info(
                        f'[OUTPUT Twist {self._out_twist_topic}] +sub '
                        # f'({self._last_out_counts["twist"]} -> {c})'
                        f'({c})'
                    )
                else:
                    self.get_logger().info(
                        f'[OUTPUT Twist {self._out_twist_topic}] -sub '
                        # f'({self._last_out_counts["twist"]} -> {c})'
                        f'({c})'
                    )
                self._last_out_counts['twist'] = c

        if self.pub_stamped:
            c = self.pub_stamped.get_subscription_count()
            if c != self._last_out_counts['stamped']:
                if c > self._last_out_counts['stamped']:
                    self.get_logger().info(
                        f'[OUTPUT TwistStamped {self._out_stamped_topic}] +sub '
                        # f'({self._last_out_counts["stamped"]} -> {c})'
                        f'({c})'
                    )
                else:
                    self.get_logger().info(
                        f'[OUTPUT TwistStamped {self._out_stamped_topic}] -sub '
                        # f'({self._last_out_counts["stamped"]} -> {c})'
                        f'({c})'
                    )
                self._last_out_counts['stamped'] = c


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
