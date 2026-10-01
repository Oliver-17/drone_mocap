#!/usr/bin/env python3
# =============================================================================
#  hover_check.py —— 量「懸停到底有多穩」
#
#  用法（懸停期間，另一個終端機）：
#      ros2 run drone_mocap hover_check.py
#      ros2 run drone_mocap hover_check.py --ros-args -p duration_s:=20.0
#
#  為什麼需要它：
#      「看起來蠻穩的」沒辦法比較、也沒辦法判斷調參數有沒有變好。
#      這支把肉眼看不出來的東西變成數字：
#          漂移   —— 位置離起點多遠（肉眼要漂 10 公分才看得出來）
#          自轉   —— 機頭每秒轉多少度（磁力計沒關乾淨的典型症狀）
#          高度   —— 標準差（地面效應 vs EV_DELAY 不對，幅度差很多）
#      調完 EKF2_EV_DELAY 之後再跑一次，數字會直接告訴你有沒有變好。
#
#  它只讀資料，不會送任何指令給飛機。
# =============================================================================

import math
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)


class HoverCheck(Node):
    def __init__(self):
        super().__init__("hover_check")
        ns = self.declare_parameter("px4_namespace", "MAV1").value
        self.duration = float(self.declare_parameter("duration_s", 30.0).value)

        # PX4 出來的 topic 一律 BEST_EFFORT。用預設的 RELIABLE 會完全收不到，
        # 而且不會有任何錯誤訊息，只是安靜地沒有資料。
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(VehicleLocalPosition,
                                 f"/{ns}/fmu/out/vehicle_local_position_v1",
                                 self._on_pos, qos)
        self.rows = []
        self.t0 = None
        self.last_yaw = None
        self.yaw_unwrapped = 0.0
        print(f"量測 {self.duration:.0f} 秒（訂 /{ns}/fmu/out/vehicle_local_position_v1）…")
        print("飛機要在懸停狀態，不要下任何指令\n")

    def _on_pos(self, m):
        if not (m.xy_valid and m.z_valid):
            return
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
            self.yaw_unwrapped = float(m.heading)
            self.last_yaw = float(m.heading)
        # 機頭角會在 ±180° 跳，直接相減會出現假的 360° 變化。
        # 每一步只取「最短的那個轉法」再累加，才量得到真正的累積轉動。
        d = float(m.heading) - self.last_yaw
        d = (d + math.pi) % (2 * math.pi) - math.pi
        self.yaw_unwrapped += d
        self.last_yaw = float(m.heading)
        self.rows.append((now - self.t0, m.x, m.y, m.z, m.vx, m.vy, m.vz,
                          self.yaw_unwrapped))

    def done(self):
        return self.t0 is not None and (time.monotonic() - self.t0) >= self.duration

    def report(self):
        r = self.rows
        if len(r) < 10:
            print(f"✗ 只收到 {len(r)} 筆有效資料 —— 確認飛機的 xy_valid 是 true")
            return
        t = [x[0] for x in r]
        x = [x[1] for x in r]
        y = [x[2] for x in r]
        z = [x[3] for x in r]
        yaw = [x[7] for x in r]
        vxy = [math.hypot(a[4], a[5]) for a in r]

        def mean(v):
            return sum(v) / len(v)

        def sd(v):
            m = mean(v)
            return math.sqrt(sum((a - m) ** 2 for a in v) / len(v))

        # 漂移：相對「前三秒的平均位置」，而不是第一筆 —— 剛進懸停時還在收斂
        n0 = max(1, sum(1 for a in t if a < 3.0))
        x0, y0 = mean(x[:n0]), mean(y[:n0])
        dist = [math.hypot(a - x0, b - y0) for a, b in zip(x, y)]
        yaw_deg = [math.degrees(a - yaw[0]) for a in yaw]
        span = t[-1] - t[0]

        print(f"\n{'=' * 52}")
        print(f" 懸停品質（{span:.0f} 秒、{len(r)} 筆、{len(r)/span:.0f} Hz）")
        print(f"{'=' * 52}")
        print(f" 水平漂移   平均 {mean(dist)*100:5.1f} cm   最大 {max(dist)*100:5.1f} cm")
        print(f"            x 範圍 {(max(x)-min(x))*100:5.1f} cm   "
              f"y 範圍 {(max(y)-min(y))*100:5.1f} cm")
        print(f" 高度       平均 {-mean(z):5.2f} m   標準差 {sd(z)*100:4.1f} cm   "
              f"範圍 {-max(z):.2f}~{-min(z):.2f} m")
        print(f" 機頭轉動   累計 {yaw_deg[-1]:+6.1f}°   "
              f"速率 {yaw_deg[-1]/span:+5.2f} °/秒")
        print(f"            範圍 {min(yaw_deg):+.1f}° ~ {max(yaw_deg):+.1f}°")
        print(f" 水平速度   平均 {mean(vxy)*100:5.1f} cm/s   最大 {max(vxy)*100:5.1f} cm/s")
        print(f"{'-' * 52}")
        print(" 時間   北(m)   東(m)  高度(m)  機頭(°)  漂移(cm)")
        step = max(1, len(r) // 10)
        for i in range(0, len(r), step):
            print(f" {t[i]:4.0f}s  {x[i]:+6.2f}  {y[i]:+6.2f}   "
                  f"{-z[i]:5.2f}   {yaw_deg[i]:+6.1f}   {dist[i]*100:6.1f}")
        print(f"{'=' * 52}")
        # 給一個直接的判讀，不用自己記標準
        print(" 判讀：")
        print(f"   漂移 {'✓ 很好' if max(dist) < 0.10 else '⚠ 偏大' if max(dist) < 0.30 else '✗ 太大'}"
              f"（< 10 cm 很好、> 30 cm 要查）")
        rate = abs(yaw_deg[-1] / span)
        print(f"   自轉 {'✓ 很好' if rate < 0.5 else '⚠ 有轉' if rate < 2.0 else '✗ 明顯自轉'}"
              f"（< 0.5 °/秒很好、> 2 °/秒代表偏航來源有問題）")
        print(f"   高度 {'✓ 很好' if sd(z) < 0.03 else '⚠ 偏晃' if sd(z) < 0.08 else '✗ 振盪'}"
              f"（標準差 < 3 cm 很好）")


def main():
    rclpy.init()
    node = HoverCheck()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.report()
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
