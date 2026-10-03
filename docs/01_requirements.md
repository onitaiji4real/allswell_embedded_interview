# 01 — 需求拆解與驗收標準

> 每一項以「需求 → 驗收標準（Acceptance Criteria, AC）」呈現。AC 全部通過才算完成。
> 實作細節與 MAVLink 對照請見 [04_mavlink_reference.md](04_mavlink_reference.md)。

---

## 共通需求（適用所有 Part）

| ID | 需求 | 驗收標準 |
|---|---|---|
| C-1 | 失敗處理 | 指令被拒、逾時、連線中斷時：印出**一行人類可讀的錯誤訊息**（含失敗步驟與原因，例如 `ERROR: arm rejected: MAV_RESULT_TEMPORARILY_REJECTED`），以 `exit code != 0` 結束，**無 traceback** |
| C-2 | 不卡住 | 每個等待都有 timeout；沒有無限迴圈 |
| C-3 | 不盲睡 | 不用 `time.sleep()` 當作「動作已完成」的依據（見 [02_constraints.md](02_constraints.md)） |
| C-4 | 成功結束 | 成功時 `exit code == 0`，並關閉連線 |
| C-5 | Ctrl+C | `KeyboardInterrupt` 需被攔截，印出訊息後非零結束（加分：飛行中被中斷時嘗試下 RTL） |
| C-6 | 連線字串 | 預設 `tcp:127.0.0.1:5760`，建議可用 CLI 參數或環境變數覆寫 |

---

## Part 1 — `part1_takeoff.py`

| ID | 需求 | 驗收標準 |
|---|---|---|
| P1-1 | 連線並收到心跳 | 30 秒內收到 `HEARTBEAT`，否則錯誤結束 |
| P1-2 | 等待可飛行（EKF/GPS 收斂） | 以 `EKF_STATUS_REPORT` 旗標（及/或 `GPS_RAW_INT.fix_type >= 3`、`STATUSTEXT`）判斷就緒；開機後約 40–60 秒；timeout 建議 120 秒；等待期間定期印出狀態 |
| P1-3 | 切到 GUIDED | 送出模式切換 → 收到 `COMMAND_ACK` 為 `ACCEPTED` → **且** `HEARTBEAT.custom_mode == GUIDED` 才算成功 |
| P1-4 | 解鎖 | `MAV_CMD_COMPONENT_ARM_DISARM` 的 ACK 為 `ACCEPTED` **且** `HEARTBEAT.base_mode` 含 `MAV_MODE_FLAG_SAFETY_ARMED` |
| P1-5 | 起飛 15 m | 解鎖後**立即**送 `MAV_CMD_NAV_TAKEOFF`（約 10 秒內不起飛會自動上鎖）；ACK 需為 `ACCEPTED` |
| P1-6 | 印出爬升高度 | 使用 `GLOBAL_POSITION_INT.relative_alt`（mm → m），定期印出 |
| P1-7 | 抵達判定 | `abs(alt - 15) <= 0.5` 時印出 `REACHED` 並乾淨結束（exit 0） |
| P1-8 | 中途上鎖偵測 | 爬升過程若偵測到被上鎖或模式被改走 → 錯誤結束 |

> 備註：Part 1 結束時無人機停留在 15 m 懸停（GUIDED 下會保持位置）。重新測試前執行 `docker compose restart`。

---

## Part 2 — `part2_square.py`

| ID | 需求 | 驗收標準 |
|---|---|---|
| P2-1 | 起飛 | 重用 Part 1 邏輯起飛至 15 m |
| P2-2 | 正方形航點 | 以**起飛後當下位置**（或 Home）為起始角點 A，計算 B/C/D 使每邊 80 m，順序 A→B→C→D→A |
| P2-3 | 前往角點 | 使用 `SET_POSITION_TARGET_GLOBAL_INT`（建議）送出目標，高度維持 15 m（相對 Home） |
| P2-4 | 抵達判定 | **水平**距離 ≤ 2 m 視為抵達（不考慮高度差） |
| P2-5 | 進度輸出 | 飛行中約 **1 Hz** 印出「目前第幾個角點、距離幾公尺」 |
| P2-6 | 每段 timeout | 每段航程設 timeout（建議 80 m ÷ 預設速度約 5 m/s ≈ 16 s，取 60–90 s 保守值） |
| P2-7 | 返航 | 回到 A 後切 RTL（ACK + HEARTBEAT 確認） |
| P2-8 | 等待落地上鎖 | 等到 `HEARTBEAT` 顯示**已上鎖**（可輔以 `EXTENDED_SYS_STATE.landed_state == ON_GROUND`）後 exit 0；timeout 建議 180 s |

---

## Part 3 — `part3_failsafe.py`

| ID | 需求 | 驗收標準 |
|---|---|---|
| P3-1 | 啟動任務 | 與 Part 2 相同的正方形任務 |
| P3-2 | 並行監控 | 任務進行中**同時**監控 `SYS_STATUS.voltage_battery`（mV；`65535`/`UINT16_MAX` 表示未知要忽略） |
| P3-3 | 觸發條件 | 電壓 `< 11.0 V` |
| P3-4 | 中止動作 | 立即：(a) 停止送 goto（任務邏輯不得再送任何位置目標）、(b) 切 RTL 並確認、(c) 記錄**含時間戳記**的中止原因（例如 `2026-10-04T02:51:07.123 ABORT: battery 10.50 V < 11.00 V`） |
| P3-5 | 自行注入故障 | 任務開始**約 20 秒**後，用 MAVLink 參數協定 `PARAM_SET` 將 `SIM_BATT_VOLTAGE` 設為 `10.5` |
| P3-6 | 確認參數 | 收到 `PARAM_VALUE`，`param_id == "SIM_BATT_VOLTAGE"` 且值 ≈ 10.5（容差 0.01），否則重試（建議最多 3 次）後錯誤結束 |
| P3-7 | 降落結束 | RTL 後等待落地且上鎖 → exit 0（因為「中止並安全返航」是此腳本的**預期成功路徑**） |
| P3-8 | 日誌說故事 | 輸出依序呈現：任務開始 → 參數注入（已確認）→ 偵測低電壓 → 中止 → RTL → 落地上鎖 → 結束。每行帶時間戳記 |
| P3-9 | 設計說明 | 在 `NOTES.md` 說明選擇 threads / asyncio / 單迴圈狀態機的理由 |

> 「約 20 秒」的計時起點請在程式與 `NOTES.md` 中明確定義。建議：**從開始飛正方形第一段時起算**，此時約位於第 1～2 邊，符合「mid-square」。
> 若沒有偵測到低電壓就飛完了正方形，應視為**失敗**（非零結束），因為代表監控器沒作用。

---

## Bonus（擇一，選做）

| 選項 | 驗收標準 |
|---|---|
| B-1 AUTO 任務 | 用 mission protocol（`MISSION_COUNT` → 回應 `MISSION_REQUEST_INT` → 送 `MISSION_ITEM_INT` → 收 `MISSION_ACK`）上傳：TAKEOFF + 4～5 個 WAYPOINT + RTL；切 AUTO 飛完並落地 |
| B-2 Geofence | 以 Home 為圓心定義半徑（例如 100 m）；接近（例如 80%）時警告；越界時強制 RTL 並記錄 |

---

## 交付項目

| ID | 項目 | 驗收標準 |
|---|---|---|
| D-1 | `part1_takeoff.py`、`part2_square.py`、`part3_failsafe.py` | 位於 repo 根目錄，可直接 `python partX_*.py` 執行 |
| D-2 | 共用模組 | 例如 `starter/drone.py`、`starter/geo.py` |
| D-3 | 測試（建議） | `pytest` 可在**不需 SITL** 的情況下跑過的單元測試（例如座標計算、ACK 解析） |
| D-4 | `NOTES.md` | 約半頁英文（原題為英文面試，建議用英文撰寫）：設計決策、更多時間會改什麼、意外發現 |
| D-5 | Git | 以 git repo 連結交付，commit 歷史清楚 |
