# =============================================================================
#  mocap.launch.py — 動捕進 PX4：VRPN 客戶端 + 座標橋接
#
#  用法（樹莓派上，和 MicroXRCEAgent 同一台）：
#      ros2 launch drone_mocap mocap.launch.py namespace:=MAV1
#      ros2 launch drone_mocap mocap.launch.py namespace:=MAV1 vrpn:=false   # vrpn 已經自己開了
#      ros2 launch drone_mocap mocap.launch.py vrpn:=false fake:=true        # 在家用假資料測
#
#  ---------------------------------------------------------------------------
#  為什麼整套都放在樹莓派上（而不是地面站）
#  ---------------------------------------------------------------------------
#  VRPN 是單純的 client-server：給 IP 和埠就連，連不上會直接報錯。
#  DDS 則要先「發現」對方，預設走多播，而飛場的 WiFi 擋多播 ——
#  2026-09-30 實測，地面站和樹莓派之間的 DDS 完全不通，動捕資料從來沒到過飛控，
#  而且全程沒有任何錯誤訊息。
#
#  把 vrpn client 和橋接都搬到樹莓派之後：
#      Motive ──VRPN(WiFi)──> 樹莓派（vrpn + 橋接 + agent）──序列埠──> 飛控
#  跨機的只剩一段單純的 client-server，而且地面站掛掉不影響飛行。
#  這和 real_nav2.launch.py 檔頭「控制迴路整條都不經過 WiFi」是同一個原則 ——
#  位置估計比控制指令更關鍵，本來就該一起搬。
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration("namespace")
    # 剛體在 Motive 裡的名字。預設和 namespace 同名（MAV1 的剛體就叫 MAV1），
    # 兩者不一樣時再用 rigid_body 覆蓋。
    rb = LaunchConfiguration("rigid_body")
    mocap_topic = PythonExpression(["'/vrpn_mocap/' + '", rb, "' + '/pose'"])
    px4_topic = PythonExpression(["'/' + '", ns, "' + '/fmu/in/vehicle_visual_odometry'"])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="MAV1",
                              description="PX4 的 namespace"),
        DeclareLaunchArgument("rigid_body", default_value="MAV1",
                              description="Motive 裡剛體的名字"),
        DeclareLaunchArgument("vrpn", default_value="true",
                              description="要不要一起開 VRPN 客戶端"),
        DeclareLaunchArgument("vrpn_server", default_value="192.168.1.2",
                              description="跑 Motive 那台電腦的 IP"),
        DeclareLaunchArgument("vrpn_port", default_value="3883",
                              description="Motive 的 VRPN 串流埠"),
        DeclareLaunchArgument("source_frame", default_value="enu",
                              description="動捕的座標系：enu（Motive 設 Z-up）/ nwu / passthrough"),
        DeclareLaunchArgument("max_rate_hz", default_value="100.0",
                              description="送給飛控的上限（EKF2 只需要 >5 Hz）"),
        DeclareLaunchArgument("fake", default_value="false",
                              description="true = 改用假動捕資料（在家測用）"),
        DeclareLaunchArgument("fake_mode", default_value="still",
                              description="still / circle / line"),

        # VRPN 客戶端。用 ExecuteProcess 而不是 IncludeLaunchDescription，
        # 因為後者會在「組 launch 描述」時就去找 vrpn_mocap 的路徑 ——
        # 開發機上沒裝那個套件，連 vrpn:=false 都會直接失敗。
        ExecuteProcess(
            cmd=["ros2", "launch", "vrpn_mocap", "client.launch.yaml",
                 ["server:=", LaunchConfiguration("vrpn_server")],
                 ["port:=", LaunchConfiguration("vrpn_port")]],
            output="screen",
            condition=IfCondition(LaunchConfiguration("vrpn")),
        ),

        Node(package="drone_mocap", executable="fake_mocap_publisher.py",
             name="fake_mocap", output="screen", emulate_tty=True,
             parameters=[{"topic": mocap_topic,
                          "mode": LaunchConfiguration("fake_mode")}],
             condition=IfCondition(LaunchConfiguration("fake"))),

        Node(package="drone_mocap", executable="mocap_to_px4_node.py",
             name="mocap_to_px4", output="screen", emulate_tty=True,
             parameters=[{"mocap_topic": mocap_topic,
                          "px4_topic": px4_topic,
                          "source_frame": LaunchConfiguration("source_frame"),
                          "max_rate_hz": LaunchConfiguration("max_rate_hz")}]),
    ])
