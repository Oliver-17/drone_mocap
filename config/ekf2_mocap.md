# 室內動捕的 PX4 參數

飛控裡的設定，**不是** ROS 的參數 —— 用 QGC 的
`Vehicle Setup → Parameters` 改，或 MAVLink Console 的 `param set`。
每一架飛機各設一份，改完 `param save`、重開飛控。

> 改之前先備份：`QGC → Parameters → Tools → Save to file`。
> 那台飛機現在的設定沒有任何備份，任何誤操作或重刷韌體都會全部消失。

## 要設的五個

| 參數 | 設成 | 為什麼 | 不設會怎樣 |
|---|---|---|---|
| `EKF2_EV_CTRL` | `11` | bit0 水平位置(1) + bit1 垂直位置(2) + bit3 偏航(8)。<br>**偏航一定要開**：橋接送的 `pose_frame` 是 FRD，意思是「這個座標系和真北差一個未知角度」，開了偏航融合 EKF2 才會把自己對齊過來（`ev_yaw_control.cpp:172-183`）。不開的話它會改用另一組估計的旋轉，而那需要磁力計 —— 但我們正要關掉磁力計 | 完全不使用視覺，資料送到就被丟掉 |
| `EKF2_HGT_REF` | `3`（Vision） | 高度以動捕為準 | 退回氣壓計。室內開門、冷氣、有人走過都會讓它跳幾十公分 |
| `EKF2_GPS_CTRL` | `0` | 室內收不到 GPS | EKF 一直嘗試 GPS，偶爾收到幾顆星就把位置拉走 |
| `EKF2_MAG_TYPE` | `5`（None） | 偏航由動捕提供，比磁力計準得多 | 磁力計被鋼筋、馬達電流干擾，和動捕的偏航打架 → **懸停時緩慢自轉或畫圈** |
| `EKF2_EV_DELAY` | `40`（ms） | 整條鏈的延遲：相機曝光 → Motive → VRPN → 網路 → agent → 飛控。**改完要重開飛控**（`params_external_vision.yaml:25` 標了 `reboot_required`） | 把 40 ms 前的位置當成現在 → 一移動就超調、來回擺 |

## 不用動的

| 參數 | 預設 | 為什麼不用改 |
|---|---|---|
| `EKF2_EV_QMIN` | `0` | 0 = 不檢查品質。橋接沒有填 `quality`（動捕也沒有這個概念），保持 0 才不會全部被擋掉 |
| `EKF2_EV_NOISE_MD` | `0` | 0 = 用訊息裡的變異數。橋接送 0，而 EKF2 會取 `max(EKF2_EVP_NOISE², 收到的值)`，等同於用參數值（0.1 m） |

## 改完怎麼確認

```bash
# 1. 參數（QGC MAVLink Console）
param show EKF2_EV_CTRL

# 2. 有沒有真的在融合（和 agent 同一台，通常是樹莓派）
ros2 topic echo /MAV1/fmu/out/estimator_status_flags --once | grep -E "cs_ev_"
#    cs_ev_pos / cs_ev_yaw / cs_ev_hgt 都要是 true

# 3. 位置估計有效嗎
ros2 topic echo /MAV1/fmu/out/vehicle_local_position_v1 --once | grep -E "xy_valid|z_valid"
```

第 2 項如果全是 false，**先別懷疑參數** —— 2026-09-30 的經驗是參數全對，
但資料根本沒抵達飛控。先在 MAVLink Console 打 `listener vehicle_visual_odometry`，
回 `never published` 就代表問題在資料路徑（DDS、agent、topic 名字），不在 EKF2。

## 室內／室外是兩套設定

`EKF2_GPS_CTRL = 0` 和 `EKF2_MAG_TYPE = 5` 會讓這台飛機**無法在室外用 GPS 飛**。
共用飛機的話，建議存成兩份參數檔（`indoor_mocap.params` / `outdoor_gps.params`），
要換場地就 `Load from file`，不要每次手動改五個參數 —— 漏一個就會出事，而且很難察覺。
