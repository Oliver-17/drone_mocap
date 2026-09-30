#!/usr/bin/env python3
# =============================================================================
#  fake_mocap_publisher.py —— 假裝自己是動捕，發已知的 PoseStamped
#
#  用法：
#      ros2 run drone_mocap fake_mocap_publisher.py --ros-args \
#          -p topic:=/vrpn_mocap/MAV1/pose -p mode:=circle
#
#  為什麼需要它：
#      動捕在飛場、開發在家裡。沒有這支的話，橋接和 EKF2 參數要等到進場
#      才能驗，而進場時間很貴。有了它，在家就能把「參數設對了沒、轉換對了沒、
#      EKF2 會不會開始融合」全部跑過一遍。
#
#  ⚠️ 它發的 QoS 和 vrpn_mocap 一樣是 BEST_EFFORT（SensorDataQoS），
#     這樣連「QoS 不相容」這種坑也能一起驗到。
#
#  三種模式：
#      still  停在一點（驗轉換和融合會不會啟動）
#      circle 繞圈（驗移動時位置跟不跟得上）
#      line   沿東西向來回（單軸移動，最好對答案）
# =============================================================================

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class FakeMocap(Node):
    def __init__(self):
        super().__init__("fake_mocap_publisher")

        def p(name, default):
            return self.declare_parameter(name, default).value

        topic = p("topic", "/vrpn_mocap/MAV1/pose")
        self.mode = p("mode", "still")
        self.rate = float(p("rate_hz", 100.0))
        self.height = float(p("height_m", 0.1))     # 動捕的 z＝離地高度（ENU）
        self.radius = float(p("radius_m", 1.0))
        self.period = float(p("period_s", 20.0))
        self.x0 = float(p("x0", 0.0))
        self.y0 = float(p("y0", 0.0))

        self.pub = self.create_publisher(PoseStamped, topic, qos_profile_sensor_data)
        self.t = 0.0
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"假動捕啟動：{topic}，模式 {self.mode}、{self.rate:.0f} Hz、"
            f"高度 {self.height} m（ENU，z 朝上）")

    def _tick(self):
        self.t += self.dt
        w = 2.0 * math.pi / self.period

        if self.mode == "circle":
            x = self.x0 + self.radius * math.cos(w * self.t)
            y = self.y0 + self.radius * math.sin(w * self.t)
            yaw = w * self.t + math.pi / 2.0      # 機頭朝著前進方向
        elif self.mode == "line":
            x = self.x0 + self.radius * math.sin(w * self.t)
            y = self.y0
            yaw = 0.0
        else:                                      # still
            x, y, yaw = self.x0, self.y0, 0.0

        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "world"
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = self.height
        # 只有偏航，繞 ENU 的 z（朝上）轉 → ROS 順序 (x, y, z, w)
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(m)


def main():
    rclpy.init()
    node = FakeMocap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
