#!/usr/bin/env python3
# =============================================================================
#  t_frames.py —— 驗座標轉換。純數學，不需要 ROS、不需要動捕、不需要飛機
#
#  跑法：
#      python3 ~/ros2_ws/src/drone_mocap/test/t_frames.py
#
#  為什麼值得寫這個測試：
#      座標轉換錯了不會當掉、不會報錯，只會讓飛機飛去錯的地方或慢慢歪。
#      而且要驗它通常得把整套動捕+飛控都架起來 —— 太貴了。
#      把轉換抽成純函式之後，這些性質在筆電上兩秒就能驗完。
# =============================================================================

import importlib.util
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(HERE, "..", "scripts", "mocap_to_px4_node.py")


def load_module():
    """直接載入節點檔，不經過 ROS。

    節點檔在最上面 import rclpy / px4_msgs，沒有 ROS 環境會失敗，
    所以先塞假的進 sys.modules —— 我們只要那幾個純數學函式。
    """
    import types
    for name in ("rclpy", "rclpy.node", "rclpy.qos", "geometry_msgs",
                 "geometry_msgs.msg", "px4_msgs", "px4_msgs.msg"):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)
    sys.modules["rclpy.node"].Node = object
    sys.modules["rclpy.qos"].qos_profile_sensor_data = None
    sys.modules["geometry_msgs.msg"].PoseStamped = object
    sys.modules["px4_msgs.msg"].VehicleOdometry = object

    spec = importlib.util.spec_from_file_location("mocap_node", NODE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


FAILS = []


def check(name, got, want, tol=1e-6):
    ok = all(abs(g - w) <= tol for g, w in zip(got, want))
    # 四元數的 q 和 −q 是同一個旋轉，所以反號也算過
    if not ok and len(got) == 4:
        ok = all(abs(g + w) <= tol for g, w in zip(got, want))
    mark = "✓" if ok else "✗"
    got_s = " ".join(f"{v:+.3f}" for v in got)
    want_s = " ".join(f"{v:+.3f}" for v in want)
    print(f"  {mark} {name:<34} 得到 [{got_s}]  期望 [{want_s}]")
    if not ok:
        FAILS.append(name)


def main():
    m = load_module()
    s = math.sqrt(0.5)

    print("ENU → NED：位置（動捕 z 是高度）")
    # 飛機在動捕的東 2 m、北 3 m、高 1.5 m
    ned, _ = m.convert_enu((2.0, 3.0, 1.5), (0, 0, 0, 1))
    check("東2 北3 高1.5", ned, (3.0, 2.0, -1.5))          # 北=y、東=x、下=−z
    ned, _ = m.convert_enu((0.0, 0.0, 0.8), (0, 0, 0, 1))
    check("只有高度 0.8", ned, (0.0, 0.0, -0.8))
    # 「下」一定要是負的（飛機在原點上方），這條錯了重力方向就反了
    ned, _ = m.convert_enu((1.0, 1.0, 2.0), (0, 0, 0, 1))
    check("高度恆為負的 down", (ned[2],), (-2.0,))

    print("\nENU → NED：姿態")
    # 機體軸和 ENU 世界軸重合（單位四元數）＝ 機頭朝東、機腹朝下
    # 轉到 NED/FRD 之後應該是「繞下軸轉 +90°」（北轉到東）
    _, q = m.convert_enu((0, 0, 0), (0, 0, 0, 1))
    check("單位姿態 → 偏航 +90°", q, (s, 0.0, 0.0, s))

    # 機頭朝北（ENU 下繞 z 轉 +90°）→ NED 下應該是偏航 0
    q_ros_yaw90 = (0.0, 0.0, s, s)          # ROS (x,y,z,w)
    _, q = m.convert_enu((0, 0, 0), q_ros_yaw90)
    check("機頭朝北 → 偏航 0", q, (1.0, 0.0, 0.0, 0.0))

    # 機頭朝北、再抬頭 10°（繞 ENU 的 x 轉？不是，抬頭是繞機體右軸）
    # 用簡單一點的性質檢查：轉換後仍是單位四元數
    for name, qr in [("單位", (0, 0, 0, 1)), ("偏航90", q_ros_yaw90),
                     ("斜的", (0.1, 0.2, 0.3, math.sqrt(1 - 0.14)))]:
        _, q = m.convert_enu((0, 0, 0), qr)
        n = math.sqrt(sum(v * v for v in q))
        check(f"{name}：轉換後仍是單位四元數", (n,), (1.0,), tol=1e-6)

    print("\nNWU → NED（舊 bridge 的行為，留著對照）")
    ned, q = m.convert_nwu((2.0, 3.0, 1.5), (0, 0, 0, 1))
    check("繞 X 軸 180°", ned, (2.0, -3.0, -1.5))
    check("姿態同樣繞 X 軸 180°", q, (1.0, 0.0, 0.0, 0.0))

    print("\n兩種轉換的差別（這就是 map 和動捕差 90° 的來源）")
    p = (2.0, 3.0, 1.5)
    a, _ = m.convert_enu(p, (0, 0, 0, 1))
    b, _ = m.convert_nwu(p, (0, 0, 0, 1))
    print(f"    同一個動捕點 {p}")
    print(f"      正確（enu）：北={a[0]:+.1f} 東={a[1]:+.1f} 下={a[2]:+.1f}")
    print(f"      舊的（nwu）：北={b[0]:+.1f} 東={b[1]:+.1f} 下={b[2]:+.1f}")

    print("\n四元數工具")
    check("單位元", m.q_mul((1, 0, 0, 0), (0.5, 0.5, 0.5, 0.5)), (0.5, 0.5, 0.5, 0.5))
    check("和自己的逆相乘 = 單位元",
          m.q_mul((s, 0, 0, s), m.q_inv((s, 0, 0, s))), (1.0, 0.0, 0.0, 0.0))

    print()
    if FAILS:
        print(f"✗ {len(FAILS)} 項失敗：{FAILS}")
        sys.exit(1)
    print("✓ 全部通過")


if __name__ == "__main__":
    main()
