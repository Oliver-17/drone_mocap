# drone_mocap

把動作捕捉（OptiTrack / VRPN）的位姿轉成 PX4 的 `vehicle_visual_odometry`，
讓 EKF2 在**室內沒有 GPS** 的環境裡有位置來源。

- [這個套件在整體架構的位置](#這個套件在整體架構的位置)
- [快速開始](#快速開始)
- [為什麼重寫一支](#為什麼重寫一支)
- [座標系](#座標系)
- [PX4 那一側要設的參數](#px4-那一側要設的參數)
- [在家怎麼驗](#在家怎麼驗)
- [排錯](#排錯)
- [相依的三個 repo](#相依的三個-repo)

---

## 這個套件在整體架構的位置

```
[實體] 飛機上的反光球
   ↓ 紅外線
[Motive] 三角定位 → 剛體位姿
   ↓ ═══ VRPN 協定（埠 3883）═══
[vrpn_mocap] geometry_msgs/PoseStamped（動捕的世界座標）
   ↓ ─── ROS 2 ───
[drone_mocap] ← 這個套件：換座標系、補時間戳
   ↓ px4_msgs/VehicleOdometry（NED + FRD）
[MicroXRCEAgent]
   ↓ ═══ 序列埠 921600 ═══
[PX4 EKF2] 融合進位置估計 → Nav2 和所有控制才有位置可用
```

**整條鏈建議都跑在樹莓派上**（和 agent 同一台）。原因見
[`launch/mocap.launch.py`](launch/mocap.launch.py) 的檔頭：VRPN 是單純的
client-server（給 IP 就連、連不上會報錯），而 DDS 要先「發現」對方、預設走多播，
飛場的 WiFi 擋多播 —— 2026-09-30 實測，地面站和樹莓派之間的 DDS 完全不通，
動捕資料從來沒抵達過飛控，而且**全程沒有任何錯誤訊息**。

---

## 快速開始

### 實機（樹莓派上）

```bash
# 只有第一次要裝
sudo apt install ros-humble-vrpn-mocap

# agent 先開（另一個終端機）
MicroXRCEAgent serial --dev /dev/ttyS5 -b 921600

# VRPN 客戶端 + 橋接，一支 launch
ros2 launch drone_mocap mocap.launch.py namespace:=MAV1 vrpn_server:=192.168.1.2
```

### 在家驗（不需要動捕）

```bash
ros2 launch drone_mocap mocap.launch.py vrpn:=false fake:=true fake_mode:=circle
```

### 確認有沒有成功

```bash
# 這三個都要是 true
ros2 topic echo /MAV1/fmu/out/estimator_status_flags --once | grep cs_ev_
```

橋接自己每 5 秒也會報一次「收到幾筆、送出幾筆」，沒資料就會 warn。

---

## 為什麼重寫一支

既有的 `mocap_px4_bridge` 前提是「動捕輸出 **NWU**」（它做的是繞 X 軸轉 180°，
對 NWU 來源剛好正確）。我們的 Motive 設成 Z-up，輸出的是 **ENU**，
套同一個轉換會得到「東-南-下」：

| 動捕點 (x=2, y=3, z=1.5) | 轉出來的 NED |
|---|---|
| 正確的 ENU→NED | 北 +3.0、東 +2.0、下 −1.5 |
| 舊的（當成 NWU） | 北 +2.0、東 −3.0、下 −1.5 |

**能飛**（z 仍朝下、位置和姿態自洽，而且 `pose_frame=FRD` 會讓 EKF2 自動對齊），
但 `map` 座標系會和動捕差 90°，所有「從外面量進來的數字」——捲尺、Motive 讀數、
地圖檔、第二架飛機——都要心算。

舊的那支還在別人的流程裡跑著，所以開新的、不動舊的。

這支另外多做了：

| | |
|---|---|
| 座標系用參數選 | Motive 設定改了不用改程式（`enu` / `nwu` / `passthrough`） |
| 時間戳預設送 0 | 讓 PX4 用「收到的當下」，避開多台機器時鐘不同步 |
| 訂閱用 `SensorDataQoS` | 直接吃 vrpn_mocap 的原始輸出，**不需要 qos_relay** |
| 速度填 NaN | 而不是留預設的 0（見下方「排錯」） |
| **每 5 秒回報收發數量** | 沒資料就 warn ← 2026-09-30 最大的教訓 |

---

## 座標系

```
q_px4 = q_(ENU→NED) ⊗ q_mocap ⊗ q_(FLU→FRD)⁻¹
         換世界座標系          換機體座標系
位置： 北 = ENU 的 y、東 = ENU 的 x、下 = −ENU 的 z
```

權威參考是 PX4 自己的實作
`PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp:600-602`（位置）
和 `:934-950`（姿態）。

**姿態一定要和位置用同一組轉換**。只換世界不換機體的話，懸停看起來正常、
一移動就歪 90° —— 這是這類橋接最經典的錯誤。

轉換是純函式，可以不用 ROS 直接驗：

```bash
python3 ~/ros2_ws/src/drone_mocap/test/t_frames.py
```

### ⚠️ 建剛體時飛機要放正

Motive 用「建立剛體那一刻」的姿態當作該剛體的零姿態。

| 誤差 | 後果 |
|---|---|
| 世界座標系偏航差 90° | 會被 EKF2 吸收（`pose_frame=FRD` + EV 偏航融合），飛得起來 |
| **機體座標系有偏移**（建剛體時飛機是歪的） | **不會被吸收** —— EKF2 會認為「飛機真的是歪的」，懸停時傾斜、往一邊漂 |

檢查：飛機水平放地上，`/vrpn_mocap/MAV1/pose` 的四元數應該接近 `(w≈1, x≈0, y≈0, z≈0)`。

---

## PX4 那一側要設的參數

完整清單和理由在 [`config/ekf2_mocap.md`](config/ekf2_mocap.md)。摘要：

```
EKF2_EV_CTRL   = 11   水平位置 + 垂直位置 + 偏航
EKF2_HGT_REF   = 3    高度以視覺為準
EKF2_BARO_CTRL = 0    關掉氣壓計，否則會和動捕搶高度
EKF2_GPS_CTRL  = 0    室內沒 GPS
EKF2_MAG_TYPE  = 5    偏航交給動捕，室內磁力計不可信
EKF2_EV_DELAY  = 40   ms，改完要重開飛控
```

`EKF2_BARO_CTRL` 是 SITL 排練時量出來的：不關的話高度會停在
−1.24 m 而不是動捕說的 −1.50 m，因為氣壓計說「在地上」、動捕說「在 1.5 m」，
EKF 在兩者之間拉扯。關掉之後精準到 −1.50。

---

## 在家怎麼驗

不需要動捕、不需要真飛機，SITL 就能把整條鏈跑一遍：

```bash
# 1. 開 SITL（空地圖就好）
WORLD=empty SIM_MODEL=x500 ~/ros2_ws/src/drone_nodered/scripts/start_empty_sitl.sh

# 2. agent
MicroXRCEAgent udp4 -p 8888

# 3. 設參數（SITL 專用的 CLI，對執行中的實例直接改）
P=~/PX4-Autopilot/build/px4_sitl_default/bin/px4-param
$P --instance 0 set EKF2_EV_CTRL 11
$P --instance 0 set EKF2_HGT_REF 3
$P --instance 0 set EKF2_BARO_CTRL 0
$P --instance 0 set EKF2_GPS_CTRL 0
$P --instance 0 set EKF2_MAG_TYPE 5

# 4. 假動捕 + 橋接
ros2 launch drone_mocap mocap.launch.py vrpn:=false fake:=true fake_mode:=line

# 5. 驗收
ros2 topic echo /MAV1/fmu/out/estimator_status_flags --once | grep cs_ev_
ros2 topic echo /MAV1/fmu/out/vehicle_local_position_v1 --once | grep -E "^(x|y|z):"
```

⚠️ 步驟 3 要在模擬**啟動完成之後**做 —— `start_empty_sitl.sh` 的
`fix_preflight_params` 會設 `EKF2_MAG_TYPE 0`，太早設會被蓋掉。

實測結果（假動捕在 東2 北3 高1.5）：

```
cs_ev_pos: true   cs_ev_yaw: true   cs_ev_hgt: true
x: 3.000   y: 2.000   z: -1.500      ← 完全符合期望
```

---

## 排錯

| 症狀 | 先查 |
|---|---|
| 橋接說「沒有收到動捕資料」 | vrpn client 有沒有連上 Motive（`VRPN connection is bad`）、剛體名字對不對（`ros2 topic list \| grep vrpn`） |
| 橋接有在送，但飛控 `listener vehicle_visual_odometry` 說 `never published` | 資料沒抵達飛控。橋接和 agent 是不是同一台？跨機的話 DDS 通不通？ |
| 收得到但 `cs_ev_*` 全 false | `EKF2_EV_CTRL` 是不是 0；頻率有沒有 >5 Hz（`EV_MAX_INTERVAL` = 200 ms） |
| 融合起來但懸停會慢慢自轉／畫圈 | `EKF2_MAG_TYPE` 沒設 5，磁力計和動捕的偏航在打架 |
| 高度和動捕對不上 | `EKF2_BARO_CTRL` 沒設 0 |
| 一移動就超調、來回擺 | `EKF2_EV_DELAY` 還是 0 |
| 懸停正常、一移動就歪 90° | 位置和姿態用了不同的轉換（這支不會，但改的時候要小心） |
| 飛機以為自己是歪的，往一邊漂 | Motive 裡建剛體時飛機沒放平 |

**為什麼速度要填 NaN**：`VehicleOdometry` 的欄位預設是 0（不是 NaN）。
如果留 0 而有人打開 `EKF2_EV_CTRL` 的 bit2（3D 速度融合），EKF 就會收到
「速度恆為 0」的量測，飛機一動就和 IMU 打架、位置估計崩潰。填 NaN 之後
就算誤開那個位元也只是不融合。

---

## 相依的三個 repo

這個套件只負責「動捕 → PX4」這一段，要飛起來還需要：

| repo | 負責 |
|---|---|
| [drone_control](https://github.com/oliver920906/drone_control) | PX4 ↔ ROS 的橋接（TF、cmd_vel）、起飛降落 |
| [drone_nav2_apriltag](https://github.com/oliver920906/drone_nav2_apriltag) | Nav2 導航、拓樸地圖、AprilTag |
| [drone_bringup](https://github.com/oliver920906/drone_bringup) | 把上面兩個兜起來的實機 launch 和參數 |

執行順序（樹莓派上）：

```
MicroXRCEAgent → drone_mocap（這個）→ 確認 cs_ev_pos = true → drone_bringup 的 real_nav2
```

**`cs_ev_pos` 沒有變 true 之前不要起飛** —— 那代表位置是慣性推算出來的，
會漂，Nav2 會完全錯亂。
