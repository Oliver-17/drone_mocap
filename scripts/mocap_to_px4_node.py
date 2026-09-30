#!/usr/bin/env python3
# =============================================================================
#  mocap_to_px4_node.py —— 把動作捕捉的位姿餵給 PX4 的 EKF2
#
#  用法（樹莓派上，和 agent 同一台）：
#      ros2 run drone_mocap mocap_to_px4_node.py --ros-args \
#          -p mocap_topic:=/vrpn_mocap/MAV1/pose \
#          -p px4_topic:=/MAV1/fmu/in/vehicle_visual_odometry
#      或用 launch：ros2 launch drone_mocap mocap.launch.py namespace:=MAV1
#
#  訂 geometry_msgs/PoseStamped（vrpn_mocap 發的，動捕的世界座標）
#  發 px4_msgs/VehicleOdometry（PX4 的 NED 世界座標 + FRD 機體座標）
#
#  ---------------------------------------------------------------------------
#  為什麼重寫一支，而不是改既有的 mocap_px4_bridge
#  ---------------------------------------------------------------------------
#  那支的前提是「動捕輸出 NWU」（它做的是繞 X 軸轉 180°，對 NWU 來源剛好正確）。
#  我們的 Motive 設成 Z-up，輸出的是 ENU，套同一個轉換會得到「東-南-下」——
#  能飛（Z 仍朝下、位置和姿態自洽），但 map 座標系會和動捕差 90°，
#  所有「從外面量進來的數字」都要心算。
#  那支還在別人的流程裡跑著，所以開新的、不動舊的。
#
#  ---------------------------------------------------------------------------
#  這支多做了什麼
#  ---------------------------------------------------------------------------
#  1. 座標系用參數選（enu / nwu / passthrough），Motive 設定改了不用改程式
#  2. 時間戳預設送 0 —— 讓 PX4 用「收到的當下」，避開多台機器時鐘不同步
#  3. 訂閱用 SensorDataQoS，直接吃 vrpn_mocap 的原始輸出（不需要 qos_relay）
#  4. 速度欄位填 NaN，而不是留預設的 0
#  5. **每 5 秒回報收發數量**，沒資料就 warn
#     ← 2026-09-30 花了一個下午才查出「資料根本沒到飛控」，
#        因為這條鏈上沒有任何一段會在斷掉時出聲。這是那天最大的教訓。
# =============================================================================

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

NAN = float("nan")

# --- 四元數工具 ---------------------------------------------------------------
# 一律用 PX4 的順序 (w, x, y, z)。ROS 的訊息是 (x, y, z, w)，進出口各轉一次。
# ⚠️ 順序填反是這類橋接最常見的錯，而且症狀是「懸停正常、一動就歪」，
#    很難聯想到是四元數。所以這裡只在最外層做一次轉換，中間全部用同一種順序。


def q_mul(a, b):
    """兩個四元數相乘（w, x, y, z）。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_inv(q):
    """單位四元數的逆 = 共軛。動捕給的一定是單位四元數，不用除以模長。"""
    return (q[0], -q[1], -q[2], -q[3])


# 座標系之間的固定旋轉，抄自 PX4 自己的實作（權威參考）：
#   PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp:934-950
#     q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.Inverse()
#   註解原文：「ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI
#              rotation about X (old East/new North). This rotation is
#              symmetric, so q_ENU_to_NED == q_NED_to_ENU.」
Q_ENU_TO_NED = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
Q_FLU_TO_FRD = (0.0, 1.0, 0.0, 0.0)


def convert_enu(pos, q_ros):
    """ENU 世界 + FLU 機體 → NED 世界 + FRD 機體。這是標準情況（Motive 設 Z-up）。

    位置：北 = ENU 的 y、東 = ENU 的 x、下 = −ENU 的 z
          （GZBridge.cpp:600-602 做的就是這三行）
    姿態：世界和機體**都要換**，所以是左右各夾一個四元數。
          只換世界不換機體的話，懸停看起來正常、一移動就歪 90°。
    """
    ned = (pos[1], pos[0], -pos[2])
    q_flu_enu = (q_ros[3], q_ros[0], q_ros[1], q_ros[2])      # ROS(x,y,z,w) → (w,x,y,z)
    q = q_mul(q_mul(Q_ENU_TO_NED, q_flu_enu), q_inv(Q_FLU_TO_FRD))
    return ned, q


def convert_nwu(pos, q_ros):
    """NWU 世界 → NED。等同於「繞 X 軸轉 180°」，也就是舊 bridge 的行為。

    留著這個選項是為了**對照**：懷疑座標系搞錯時，切過去跑一次就知道
    新舊兩版的差別在哪，不用重編譯或改程式。
    """
    ned = (pos[0], -pos[1], -pos[2])
    q = (q_ros[3], q_ros[0], -q_ros[1], -q_ros[2])
    return ned, q


def convert_passthrough(pos, q_ros):
    """完全不轉換。只在除錯時用 —— 想確認「PX4 收到的到底是不是我送的」。"""
    return tuple(pos), (q_ros[3], q_ros[0], q_ros[1], q_ros[2])


CONVERTERS = {
    "enu": convert_enu,
    "nwu": convert_nwu,
    "passthrough": convert_passthrough,
}


class MocapToPx4(Node):
    def __init__(self):
        super().__init__("mocap_to_px4_node")

        def p(name, default):
            return self.declare_parameter(name, default).value

        self.mocap_topic = p("mocap_topic", "/vrpn_mocap/MAV1/pose")
        px4_topic = p("px4_topic", "/MAV1/fmu/in/vehicle_visual_odometry")

        # 來源座標系。Motive 的「Up Axis」設成 Z-up 時輸出就是 ENU（預設值）。
        frame = p("source_frame", "enu")
        if frame not in CONVERTERS:
            raise ValueError(f"source_frame 只能是 {list(CONVERTERS)}，收到 {frame!r}")
        self.frame = frame
        self.convert = CONVERTERS[frame]

        # arrival：timestamp 送 0，PX4 會蓋成「收到的當下」
        #   （ucdr/vehicle_odometry.h:117 `if (timestamp == 0) timestamp = hrt_absolute_time();`）
        # source ：用動捕訊息自己的時間戳
        # ⚠️ 預設用 arrival，因為 vrpn_mocap 的時間戳是**它那台電腦**的 ROS 時鐘
        #    （tracker.cpp:110），而 uXRCE-DDS 的時間校正只校「飛控 ↔ agent」。
        #    跨機時鐘差幾百毫秒，EV 量測就會被當成很舊的資料而失效，
        #    而且完全不會報錯。真實延遲統一用 EKF2_EV_DELAY 一個參數補。
        self.ts_mode = p("timestamp_mode", "arrival")

        # POSE_FRAME_FRD = 2：告訴 EKF2「這個座標系和真北差一個固定但未知的角度」。
        # 室內動捕的座標系本來就不對齊真北，所以這才是正確的選擇 ——
        # EKF2 開啟 EV 偏航融合後會把自己對齊過來（ev_yaw_control.cpp:172-183）。
        # 只有動捕的 x 軸真的指向北時才該用 ned。
        self.pose_frame = (VehicleOdometry.POSE_FRAME_NED
                           if p("pose_frame", "frd") == "ned"
                           else VehicleOdometry.POSE_FRAME_FRD)

        # 動捕常跑到 120~240 Hz，但 EKF2 只需要 >5 Hz（EV_MAX_INTERVAL = 200 ms，
        # common.h:71）。全部往序列埠送會和其他訊息搶頻寬，所以預設限在 100 Hz。
        rate = float(p("max_rate_hz", 100.0))
        self.min_interval = (1.0 / rate) if rate > 0 else 0.0

        self.report_period = float(p("report_period_s", 5.0))
        self.warn_after = float(p("warn_after_s", 1.0))

        # 訂閱一定要用 BEST_EFFORT：vrpn_mocap 預設就是 SensorDataQoS
        # （tracker.cpp:55），用預設的 RELIABLE 訂閱會**完全收不到而且不報錯**。
        # 舊架構為此多開了一支 qos_relay.py 轉 QoS，這裡直接對上就不用了。
        self.create_subscription(PoseStamped, self.mocap_topic, self._on_pose,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(VehicleOdometry, px4_topic, 10)

        self._rx = 0            # 這個回報週期內收到幾筆
        self._tx = 0            # 送出幾筆（被限流擋掉的不算）
        self._t_last_rx = None  # 最後一筆的時間（monotonic）
        self._next_tx = 0.0     # 下一次允許送出的時刻
        self._first = True
        self.create_timer(self.report_period, self._report)

        self.get_logger().info(
            f"動捕橋接啟動：{self.mocap_topic} → {px4_topic}\n"
            f"  來源座標系 {frame}、時間戳 {self.ts_mode}、"
            f"上限 {rate:.0f} Hz、pose_frame "
            f"{'NED' if self.pose_frame == VehicleOdometry.POSE_FRAME_NED else 'FRD'}")

    def _on_pose(self, msg):
        self._rx += 1
        now = time.monotonic()
        self._t_last_rx = now

        # 限流：EKF2 只要 >5 Hz，送太快只是佔用序列埠頻寬。
        # ⚠️ 用「下一次允許送出的時刻」而不是「距離上次多久」——
        #    後者在「來源頻率和上限相同」時會因為抖動砍掉約 1/3 的訊息
        #    （2026-09-30 SITL 實測：進來 100 Hz、上限 100 Hz，只送出 63 Hz）。
        #    每次把期限往後推固定一格，抖動就不會累積成漏送。
        if self.min_interval:
            if now < self._next_tx:
                return
            self._next_tx = max(now, self._next_tx + self.min_interval)

        pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        q_ros = (msg.pose.orientation.x, msg.pose.orientation.y,
                 msg.pose.orientation.z, msg.pose.orientation.w)
        ned, q = self.convert(pos, q_ros)

        out = VehicleOdometry()
        out.pose_frame = self.pose_frame
        if self.ts_mode == "source":
            us = int(msg.header.stamp.sec) * 1_000_000 + int(msg.header.stamp.nanosec) // 1000
            out.timestamp = us
            out.timestamp_sample = us
        else:
            out.timestamp = 0          # 0 = 請 PX4 用收到的當下
            out.timestamp_sample = 0

        out.position = [float(v) for v in ned]
        out.q = [float(v) for v in q]

        # 速度我們沒有可靠來源（動捕的速度是位置差分，雜訊大），一律填 NaN。
        # 訊息定義就是這樣寫的：「[@invalid NaN If invalid/unknown]」。
        # 留預設的 0 的話，哪天有人打開 EKF2_EV_CTRL 的 bit2（3D 速度融合），
        # EKF 就會收到「速度恆為 0」的量測，飛機一動就和 IMU 打架。
        out.velocity = [NAN, NAN, NAN]
        out.angular_velocity = [NAN, NAN, NAN]
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN

        # 變異數留 0：EKF2 會取 max(EKF2_EVP_NOISE², 收到的值)（EKF2.cpp:2290 附近），
        # 所以 0 等同於「用參數檔裡的值」，不需要我們自己編一個數字。
        self.pub.publish(out)
        self._tx += 1

        if self._first:
            self._first = False
            self.get_logger().info(
                f"第一筆資料（拿來對答案用）：\n"
                f"  動捕 {self.frame}：x={pos[0]:+.3f} y={pos[1]:+.3f} z={pos[2]:+.3f}\n"
                f"  送出 NED    ：北={ned[0]:+.3f} 東={ned[1]:+.3f} 下={ned[2]:+.3f}"
                f"（離地 {-ned[2]:+.3f} m）\n"
                f"  ⚠️ 「離地」如果不像高度，來源座標系八成設錯了")

    def _report(self):
        """每 5 秒講一次話。這條鏈上任何一段斷掉都是安靜的，所以一定要有人出聲。"""
        age = None if self._t_last_rx is None else time.monotonic() - self._t_last_rx
        if self._rx == 0:
            self.get_logger().warn(
                f"沒有收到動捕資料（{self.mocap_topic}）"
                + ("，從啟動到現在都沒有" if age is None else f"，最後一筆在 {age:.0f} 秒前")
                + " —— 檢查 vrpn client 有沒有連上 Motive、剛體名字對不對")
            return
        hz = self._rx / self.report_period
        note = ""
        if hz < 5.0:
            # EV_MAX_INTERVAL = 200 ms（common.h:71）：兩筆間隔超過就不符合融合條件
            note = "  ⚠️ 低於 5 Hz，EKF2 不會融合"
        self.get_logger().info(
            f"最近 {self.report_period:.0f} 秒：收到 {self._rx} 筆（{hz:.0f} Hz）、"
            f"送出 {self._tx} 筆{note}")
        self._rx = 0
        self._tx = 0


def main():
    rclpy.init()
    node = MocapToPx4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
