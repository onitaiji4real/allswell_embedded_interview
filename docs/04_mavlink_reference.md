# 04 — MAVLink 技術速查與已知陷阱

> 環境：ArduCopter **4.5.7** SITL。官方訊息定義：<https://mavlink.io/en/messages/common.html>
> 下列數值請在實作時再以 `pymavlink.mavutil.mavlink.*` 常數為準，**不要寫死魔術數字**。

## 1. ArduCopter 模式編號（`HEARTBEAT.custom_mode`）

| 模式 | 編號 | 用途 |
|---|---|---|
| STABILIZE | 0 | 開機預設 |
| AUTO | 3 | Bonus B-1 |
| GUIDED | 4 | Part 1–3 |
| LOITER | 5 | — |
| RTL | 6 | 返航 |
| LAND | 9 | 可能由自駕儀自身 failsafe 觸發 |

取得對照表：`conn.mode_mapping()` → `{"GUIDED": 4, ...}`。目前模式字串：`mavutil.mode_string_v10(heartbeat)`。

> ⚠️ 只採用來自**自駕儀**（`msg.get_srcSystem() == target_system` 且 `get_srcComponent() == 1`）的 HEARTBEAT；若連著地面站，也可能收到 GCS 的心跳（`type == MAV_TYPE_GCS`），要過濾掉。

## 2. 指令（`COMMAND_LONG` → `COMMAND_ACK`）

| 動作 | 指令 | 參數 |
|---|---|---|
| 切模式 | `MAV_CMD_DO_SET_MODE` (176) | p1=`MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`(1)、p2=custom_mode |
| 解鎖/上鎖 | `MAV_CMD_COMPONENT_ARM_DISARM` (400) | p1=1 解鎖 / 0 上鎖（p2=21196 強制，**不要用**） |
| 起飛 | `MAV_CMD_NAV_TAKEOFF` (22) | p7=高度（m，相對 Home） |
| 請求訊息頻率 | `MAV_CMD_SET_MESSAGE_INTERVAL` (511) | p1=msg id、p2=間隔 µs（可選，用於確保 `EKF_STATUS_REPORT`/`EXTENDED_SYS_STATE` 有串流） |

送出：
```python
conn.mav.command_long_send(target_system, target_component, cmd, confirmation, p1, p2, p3, p4, p5, p6, p7)
```

`COMMAND_ACK.result`（`MAV_RESULT`）：

| 值 | 名稱 | 處理 |
|---|---|---|
| 0 | ACCEPTED | 成功（仍需遙測確認） |
| 1 | TEMPORARILY_REJECTED | 失敗（常見：EKF 尚未就緒就解鎖） |
| 2 | DENIED | 失敗 |
| 3 | UNSUPPORTED | 失敗 |
| 4 | FAILED | 失敗（常見：prearm 檢查未過） |
| 5 | IN_PROGRESS | 繼續等待 |

- 只接受 `ack.command == 送出的 cmd` 的 ACK。
- 被拒時，ArduPilot 通常會同時送 `STATUSTEXT`（例如 `PreArm: ...`、`Arm: ...`），**把最近的 STATUSTEXT 附在錯誤訊息裡**，非常有助除錯。

## 3. 就緒判斷（Part 1 步驟 1）

`EKF_STATUS_REPORT.flags`（`EKF_STATUS_FLAGS`）：

| 位元 | 名稱 |
|---|---|
| 1 | `EKF_ATTITUDE` |
| 2 | `EKF_VELOCITY_HORIZ` |
| 4 | `EKF_VELOCITY_VERT` |
| 8 | `EKF_POS_HORIZ_REL` |
| 16 | `EKF_POS_HORIZ_ABS` |
| 32 | `EKF_POS_VERT_ABS` |
| 128 | `EKF_CONST_POS_MODE`（**必須為 0**，代表沒有位置來源） |
| 1024 | `EKF_UNINITIALIZED`（必須為 0） |
| 32768 | `EKF_GPS_GLITCH`（必須為 0） |

建議就緒條件（全部成立）：
1. `flags` 包含 `ATTITUDE | VELOCITY_HORIZ | VELOCITY_VERT | POS_HORIZ_REL | POS_HORIZ_ABS | POS_VERT_ABS`
2. `flags` 不含 `CONST_POS_MODE`、`UNINITIALIZED`、`GPS_GLITCH`
3. `GPS_RAW_INT.fix_type >= 3`
4. （可選加強）收到 `STATUSTEXT` 含 `"EKF3 IMU0 is using GPS"` 或 `"IMU1 is using GPS"`
5. （可選加強）`SYS_STATUS.onboard_control_sensors_health` 含 `MAV_SYS_STATUS_PREARM_CHECK`（ArduPilot 會回報 prearm 是否通過）

**就算判斷就緒，解鎖仍可能被拒** → 建議 `arm()` 在收到 `TEMPORARILY_REJECTED/FAILED` 時，於整體 timeout 內每隔數秒**重試**，並印出 STATUSTEXT 原因。

## 4. 位置控制（Part 2）

`SET_POSITION_TARGET_GLOBAL_INT`：
```python
conn.mav.set_position_target_global_int_send(
    0,                                   # time_boot_ms（可為 0）
    target_system, target_component,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,   # 高度相對 Home
    0b0000_1111_1111_1000,               # type_mask：只用位置（忽略速度/加速度/yaw/yaw_rate）= 0x0FF8
    int(lat * 1e7), int(lon * 1e7), alt_m,
    0, 0, 0,  0, 0, 0,  0, 0)
```
- 必須處於 **GUIDED** 模式且已解鎖並在空中。
- 無 ACK；ArduPilot 會回傳 `POSITION_TARGET_GLOBAL_INT` 可用來確認目標被接受（可選）。
- 建議每 ~1 s **重送**一次（防遺失），與 1 Hz 進度輸出同步。
- 替代方案：`MISSION_ITEM_INT` with `current=2`（guided goto）或 `MAV_CMD_DO_REPOSITION` —— 擇一即可。

位置遙測：`GLOBAL_POSITION_INT`
- `lat`/`lon`：度 × 1e7（int）
- `relative_alt`：**mm**（相對 Home）
- `alt`：mm（MSL）—— 不要拿來比 15 m

預設水平速度 `WPNAV_SPEED` ≈ 5 m/s（500 cm/s）→ 每邊 80 m 約 16–20 s，整個正方形約 70–90 s。

## 5. 降落 / 上鎖判斷

- `HEARTBEAT.base_mode & MAV_MODE_FLAG_SAFETY_ARMED == 0` → 已上鎖（主要依據）。
- `EXTENDED_SYS_STATE.landed_state == MAV_LANDED_STATE_ON_GROUND`(1)（輔助；可能需要用 `SET_MESSAGE_INTERVAL` 請求）。
- RTL 流程：爬升至 `RTL_ALT`（預設 15 m）→ 飛回 Home → 降落 → 自動上鎖。從正方形中途返航約 30–90 s。

## 6. 參數協定（Part 3）

```python
conn.mav.param_set_send(
    target_system, target_component,
    b"SIM_BATT_VOLTAGE",                 # param_id（最多 16 bytes）
    10.5,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
```
- 確認：等 `PARAM_VALUE`，`param_id`（去掉尾端 `\x00`，pymavlink 可能回 str）== `"SIM_BATT_VOLTAGE"` 且 `abs(value - 10.5) < 0.01`。
- 沒收到 → 重送（最多 3 次）；仍失敗 → 可用 `param_request_read_send` 主動讀回驗證；最後才錯誤結束。
- `PARAM_VALUE` 必須經由接收執行緒分派給等待者（不可在另一執行緒直接 `recv_match`）。

## 7. 電池（Part 3）

- `SYS_STATUS.voltage_battery`：**mV**（uint16）；`65535` = 未知 → 忽略。
- 也可參考 `BATTERY_STATUS.voltages[0]`（mV），但題目指定 `SYS_STATUS`。
- SITL 預設 `SIM_BATT_VOLTAGE` = 12.6 V；設 10.5 後下一筆 `SYS_STATUS`（約 4 Hz 時 ≤ 0.25 s）就會反映。
- 可考慮簡單去抖動（例如連續 2 筆 < 11.0 V），但題目要求「立即」，若加去抖動請在 NOTES.md 說明取捨。

## 8. 已知陷阱清單

| # | 陷阱 | 對策 |
|---|---|---|
| 1 | 開機 40–60 s 內解鎖會被拒 | 先等 EKF/GPS 就緒，解鎖失敗時在 timeout 內重試 |
| 2 | 解鎖後約 10 s 不起飛會自動上鎖 | 解鎖確認後**立刻**送 TAKEOFF |
| 3 | 必須先 GUIDED 再起飛；STABILIZE 下 TAKEOFF 會被拒 | 順序：GUIDED → arm → takeoff |
| 4 | ACK ≠ 動作完成 | 一律以遙測二次確認 |
| 5 | 多執行緒同時 `recv_match` 會搶訊息 | 單一接收執行緒（見 03） |
| 6 | `relative_alt` 單位是 mm、`lat/lon` 是 1e7 | 寫轉換函式 + 單元測試 |
| 7 | GCS（5762）連線時會收到其他系統的 HEARTBEAT | 依 srcSystem/srcComponent/type 過濾 |
| 8 | ArduPilot 自身電池 failsafe 可能也觸發（切 RTL/LAND） | 監控器仍須自行記錄並下 RTL；若模式已是 RTL/LAND，記錄後接受，不視為錯誤 |
| 9 | Part 3 後電壓維持 10.5 V，下一輪無法解鎖（電池 prearm 檢查） | 每次測試前 `docker compose restart` |
| 10 | `request_data_stream_send` 為舊式 API，某些訊息頻率可能不夠 | 必要時補 `MAV_CMD_SET_MESSAGE_INTERVAL` |
| 11 | `param_id` 可能是 bytes 或含 `\x00` 填充 | 正規化：`pid.decode() if bytes`，再 `.rstrip("\x00")` |
| 12 | 起飛 ACK 後高度一開始不變（馬達 spool-up） | 監看高度時也要同時檢查仍為 armed + GUIDED |
