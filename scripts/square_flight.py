#!/usr/bin/env python3
# =============================================================================
#  square_flight.py —— 讓實機走一個正方形（動捕定位下的第一個移動測試）
#
#  用法（樹莓派上，和 agent、mocap.launch.py 同一台）：
#      ros2 run drone_mocap square_flight.py
#      ros2 run drone_mocap square_flight.py --ros-args -p size_m:=2.0 -p speed_mps:=0.3
#
#  ⚠️ 執行前一定要確認 single_drone.launch.py **沒有在跑**。
#     PX4 只認一條 setpoint 串流，兩個節點同時發會讓飛機抽搐，
#     而且在 log 裡看起來只像控制器沒調好（drone_control 的註解裡記過這個坑）。
#
#  ---------------------------------------------------------------------------
#  它自己管全程：解鎖 → 爬升 → 走正方形 → 回中心 → 降落
#  ---------------------------------------------------------------------------
#  為什麼不是「把下一個角當目標丟給 PX4」：
#      那樣飛機會用 MPC_XY_VEL_MAX（**出廠預設 12 m/s**）全力衝過去。
#      室內 5×4 公尺的房間，那是一秒撞牆。
#  所以改成**自己把設定點用固定速度慢慢移過去** —— 飛機只是跟著一個緩慢移動的點走，
#  速度完全由這支腳本決定，不依賴飛控參數，也就不用為了安全去改 MPC_*。
#
#  ---------------------------------------------------------------------------
#  座標一律用「房間座標」（和 Motive 畫面上看到的一樣：x 東、y 北、z 上）
#  ---------------------------------------------------------------------------
#  內部才轉成 PX4 的 NED。這樣你不用在腦中轉 90°，印出來的數字也和 Motive 對得上。
#      PX4 北 = 房間 y、PX4 東 = 房間 x、PX4 下 = −房間 z
#  （和 mocap_to_px4_node 的轉換一致，所以 PX4 的本地座標系 == 動捕座標系。）
# =============================================================================

import math
import time

import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

NAN = float("nan")
RATE_HZ = 20.0          # setpoint 串流頻率。PX4 的 COM_OF_LOSS_T 預設 1 秒，
                        # 低於 2 Hz 就會掉出 offboard；20 Hz 是通用的安全值。


class SquareFlight(Node):
    def __init__(self):
        super().__init__("square_flight")

        def p(name, default):
            return self.declare_parameter(name, default).value

        ns = p("px4_namespace", "MAV1")
        self.size = float(p("size_m", 1.0))          # 正方形邊長
        self.alt = float(p("altitude_m", 0.5))       # 離起飛點多高
        self.speed = float(p("speed_mps", 0.2))      # 設定點移動速度
        self.climb = float(p("climb_mps", 0.2))      # 爬升速度
        self.dwell = float(p("dwell_s", 2.0))        # 每個角停留幾秒
        self.max_radius = float(p("max_radius_m", 2.0))   # 離起飛點的安全上限

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.pos = None
        self.status = None
        self.create_subscription(VehicleLocalPosition,
                                 f"/{ns}/fmu/out/vehicle_local_position_v1",
                                 lambda m: setattr(self, "pos", m), qos)
        self.create_subscription(VehicleStatus, f"/{ns}/fmu/out/vehicle_status_v1",
                                 lambda m: setattr(self, "status", m), qos)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, f"/{ns}/fmu/in/offboard_control_mode", 10)
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, f"/{ns}/fmu/in/trajectory_setpoint", 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, f"/{ns}/fmu/in/vehicle_command", 10)

        self.target = None      # 目前要送出去的設定點（PX4 NED）
        self.vel_ff = None      # 速度前饋（PX4 NED）。None = 不送（停在原地時）
        self.yaw = None         # 整趟鎖住的機頭角，避免飛機邊走邊轉
        self.home = None        # 起飛點（PX4 NED），所有位移都相對它
        self.aborted = False

    # --- 小工具 ---------------------------------------------------------------

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def spin(self, seconds):
        """跑 ROS 迴圈並持續送設定點。

        ⚠️ 一定要限速。spin_once 幾乎立刻返回，在迴圈裡直接發等於用數千 Hz 灌，
           PX4 的佇列會塞爆 —— 症狀是「解鎖了也進 offboard 了，就是不動」。
        """
        end = time.time() + seconds
        period, nxt = 1.0 / RATE_HZ, 0.0
        while rclpy.ok() and time.time() < end:
            t = time.time()
            if t >= nxt:
                self._send_setpoint()
                nxt = t + period
            rclpy.spin_once(self, timeout_sec=0.01)
            if self._unsafe():
                return False
        return True

    def _send_setpoint(self):
        if self.target is None:
            return
        m = OffboardControlMode()
        m.timestamp = self.now_us()
        m.position = True       # 只用位置控制，其他全部 False
        self.mode_pub.publish(m)

        s = TrajectorySetpoint()
        s.timestamp = m.timestamp
        s.position = [float(v) for v in self.target]
        # 速度前饋：和位置一起送，PX4 在 position=True 時會把它當前饋。
        # ⚠️ 沒有這個的話會有結構性的落後 —— PX4 的位置控制器是比例控制
        #    （MPC_XY_P 預設 0.95 1/s），速度指令 = 增益 × 位置誤差，
        #    所以要維持 0.2 m/s 就「必須」有 0.2/0.95 = 21 cm 的誤差。
        #    2026-10-01 實測：不送前饋時 SITL 和實機都穩定落後約 20 cm。
        # 停在原地時送 NaN（不是 0）—— 填 0 會被當成「要求速度為零」，
        # 和位置目標打架。
        s.velocity = ([float(v) for v in self.vel_ff] if self.vel_ff
                      else [NAN, NAN, NAN])
        s.acceleration = [NAN, NAN, NAN]
        s.yaw = float(self.yaw) if self.yaw is not None else NAN
        s.yawspeed = NAN
        self.sp_pub.publish(s)

    def cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = command
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True      # 少了這個 PX4 會當成內部指令直接拒絕
        m.timestamp = self.now_us()
        self.cmd_pub.publish(m)

    def _unsafe(self):
        """每一輪都檢查。有問題就記下來，由主流程去降落。"""
        if self.aborted:
            return True
        if self.pos is None or not self.pos.xy_valid or not self.pos.z_valid:
            self.get_logger().error("位置估計失效 —— 中止並降落")
            self.aborted = True
            return True
        if self.home is not None:
            d = math.hypot(self.pos.x - self.home[0], self.pos.y - self.home[1])
            if d > self.max_radius:
                self.get_logger().error(
                    f"離起飛點 {d:.2f} m 超過上限 {self.max_radius} m —— 中止並降落")
                self.aborted = True
                return True
        return False

    # --- 座標：房間（ENU）→ PX4（NED）------------------------------------------

    def room_to_ned(self, east, north, up):
        """房間座標 → PX4 的本地 NED。

        和 mocap_to_px4_node 的轉換一致，所以 PX4 的本地座標系就是動捕座標系 ——
        這裡的 (east, north) 可以直接對照 Motive 畫面上的 (x, y)。
        """
        return [self.home[0] + north, self.home[1] + east, self.home[2] - up]

    # --- 主流程 ---------------------------------------------------------------

    def move_to(self, east, north, up, label, speed=None):
        """把設定點用固定速度從現在的位置移到目標，而不是一步跳過去。

        這是整支腳本的核心安全機制：飛機永遠只是在追一個「慢慢走的點」，
        所以速度由 speed_mps 決定，不是由 MPC_XY_VEL_MAX（預設 12 m/s）決定。
        """
        goal = self.room_to_ned(east, north, up)
        start = list(self.target)
        dist = math.dist(start, goal)
        if dist < 1e-6:
            return True
        v = speed if speed is not None else self.speed
        steps = max(1, int(dist / (v / RATE_HZ)))
        # 速度前饋 = 單位方向 × 速度（PX4 NED）
        self.vel_ff = [(b - a) / dist * v for a, b in zip(start, goal)]
        # 印「絕對」房間座標。之前印相對偏移、但位置印絕對，兩個基準不同很容易誤判
        self.get_logger().info(
            f"→ {label}：房間座標 ({goal[1]:+.2f}, {goal[0]:+.2f})、"
            f"高度 {-goal[2]:.2f} m，距離 {dist:.2f} m、約 {dist / v:.0f} 秒")
        period, nxt = 1.0 / RATE_HZ, 0.0
        i = 0
        while rclpy.ok() and i < steps:
            t = time.time()
            if t >= nxt:
                i += 1
                f = i / steps
                self.target = [a + (b - a) * f for a, b in zip(start, goal)]
                self._send_setpoint()
                nxt = t + period
                if i % int(RATE_HZ) == 0 and self.pos is not None:
                    err = math.dist([self.pos.x, self.pos.y, self.pos.z], self.target)
                    self.get_logger().info(
                        f"   位置 ({self.pos.y:+.2f}, {self.pos.x:+.2f}) "
                        f"高度 {-self.pos.z:.2f}  追蹤誤差 {err * 100:.0f} cm")
            rclpy.spin_once(self, timeout_sec=0.01)
            if self._unsafe():
                self.vel_ff = None
                return False
        self.vel_ff = None      # 到點了，接下來是停留 —— 不要再叫它繼續往前
        return True

    def land(self):
        self.get_logger().info("降落…")
        self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        # NAV_LAND 會切出 offboard，由 PX4 自己管觸地偵測和上鎖。
        # 這裡只要停止送設定點、等它落地就好。
        end = time.time() + 30
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.status is not None
                    and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED):
                self.get_logger().info("已落地上鎖")
                return
        self.get_logger().warn("等了 30 秒還沒上鎖 —— 自己確認飛機狀態")

    def run(self):
        # 1) 等位置估計
        self.get_logger().info("等位置估計（動捕要在融合）…")
        end = time.time() + 30
        while rclpy.ok() and time.time() < end and (
                self.pos is None or not self.pos.xy_valid or not self.pos.z_valid):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pos is None or not self.pos.xy_valid:
            self.get_logger().error(
                "沒有有效的位置估計。確認 agent、mocap.launch.py 都在跑，"
                "而且 cs_ev_pos 是 true")
            return 1

        self.home = [self.pos.x, self.pos.y, self.pos.z]
        self.yaw = float(self.pos.heading)
        self.target = list(self.home)
        self.get_logger().info(
            f"起飛點：房間座標 ({self.home[1]:+.2f}, {self.home[0]:+.2f})、"
            f"高度 {-self.home[2]:.2f} m、機頭 {math.degrees(self.yaw):+.0f}°")
        self.get_logger().info(
            f"計畫：邊長 {self.size} m 的正方形、高度 {self.alt} m、"
            f"速度 {self.speed} m/s、每角停 {self.dwell} 秒")

        # 2) 先送一秒的設定點再切 offboard —— PX4 要求「先有串流才准切」
        if not self.spin(1.0):
            return self._bail()
        self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        if not self.spin(2.0):
            return self._bail()
        if self.status is None or self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().error("解鎖失敗 —— 看 QGC 的預檢訊息")
            return 1
        self.get_logger().info("✓ 已解鎖並進入 offboard")

        # 3) 爬升（用自己的速度，和水平移動分開）
        ok = self.move_to(0.0, 0.0, self.alt, "爬升", speed=self.climb)
        if ok:
            ok = self.spin(2.0)

        # 4) 正方形：以起飛點為中心，四個角依序走一圈，最後回中心
        h = self.size / 2.0
        corners = [(+h, +h, "東北角"), (-h, +h, "西北角"),
                   (-h, -h, "西南角"), (+h, -h, "東南角"),
                   (+h, +h, "回到東北角"), (0.0, 0.0, "回到中心")]
        for east, north, label in corners:
            if not ok:
                break
            ok = self.move_to(east, north, self.alt, label)
            if ok:
                ok = self.spin(self.dwell)

        # 5) 降落（不管前面成不成功都要降）
        self.land()
        return 0 if ok else 1

    def _bail(self):
        self.land()
        return 1


def main():
    rclpy.init()
    node = SquareFlight()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        # Ctrl+C 不是「馬上停住不管」—— 要讓飛機安全降落
        node.get_logger().warn("收到 Ctrl+C，降落")
        try:
            node.land()
        except KeyboardInterrupt:
            node.get_logger().error("再次 Ctrl+C —— 直接結束，請用遙控器接手")
        rc = 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    main()
