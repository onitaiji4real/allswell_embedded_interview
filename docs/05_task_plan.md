# 05 — 執行計畫與任務清單

> 接手的 AI 工程師：請依序執行，每完成一項就把 `[ ]` 改成 `[x]`，並在最下方「進度紀錄」追加一行。
> 每個 Phase 結束都要**在 SITL 上實際跑過**（`docker compose restart` 後執行），不是只寫完程式。

---

## Phase 0 — 環境確認

- [ ] `docker compose up -d --build`（首次約 10 分鐘）
- [ ] 建立 venv 並 `pip install -r requirements.txt`
- [ ] `python starter/telemetry.py` 在 ~20 s 內看到遙測
- [ ] （可選）QGroundControl 連 `tcp:127.0.0.1:5762` 觀察

## Phase 1 — 共用基礎（`starter/`）

- [ ] 新增 `starter/__init__.py`
- [ ] `starter/geo.py`：`offset_latlon`、`horizontal_distance_m`、`square_corners`
- [ ] `tests/test_geo.py`：80 m 偏移誤差 < 0.1 m、正方形四邊長度 ≈ 80 m、距離對稱性
- [ ] `starter/drone.py` 重構：
  - [ ] 接收執行緒（唯一 `recv_match` 呼叫者）+ `TelemetrySnapshot`（lock 保護）
  - [ ] HEARTBEAT 來源過濾（只認自駕儀）
  - [ ] `send_lock` 保護所有送出
  - [ ] 1 Hz GCS 心跳（建議）
  - [ ] `send_command_long()`：等對應 ACK、處理 IN_PROGRESS、拒絕時附 STATUSTEXT
  - [ ] `wait_until(predicate, timeout, desc)`，並檢查 `abort_event`
  - [ ] 接收執行緒在連線失效時要讓等待者得知（不可永久卡住）
  - [ ] `close()` 能乾淨停止執行緒
- [ ] `tests/test_drone_logic.py`：用 fake 連線測試 ACK 接受 / 拒絕 / 逾時路徑（不需 SITL）

## Phase 2 — Part 1（`part1_takeoff.py`）

- [ ] `wait_ready_to_arm()`（EKF 旗標 + GPS fix，timeout 120 s，期間印狀態）
- [ ] `set_mode("GUIDED")`（ACK + HEARTBEAT）
- [ ] `arm()`（ACK + HEARTBEAT，拒絕時 timeout 內重試）
- [ ] `takeoff(15)`（ACK，印高度，≤0.5 m 印 `REACHED`）
- [ ] `main()` 統一錯誤處理、退出碼
- [ ] 實機測試：正常路徑 exit 0
- [ ] 失敗測試：SITL 未啟動 → 清楚錯誤 + exit 1（無 traceback）
- [ ] 失敗測試：開機後立即執行（驗證會等待而非被拒後崩潰）

## Phase 3 — Part 2（`part2_square.py`）

- [ ] 重用 Part 1 起飛流程（抽成 `starter/mission.py` 的 `takeoff_sequence(drone, 15)`）
- [ ] 以起飛後位置為 A 計算 A→B→C→D→A
- [ ] `fly_to()`：每 ~1 s 重送目標 + 印距離，≤ 2 m 抵達，每段 timeout
- [ ] `set_mode("RTL")` + `wait_landed_disarmed()`
- [ ] 實機測試：完整飛完並落地，exit 0
- [ ] 以 QGC 目視確認軌跡為正方形（可選）

## Phase 4 — Part 3（`part3_failsafe.py`）

- [ ] `set_param()`（PARAM_SET + PARAM_VALUE 確認 + 重試）
- [ ] 電池監控 listener：`voltage < 11.0 V` → 時間戳記錄 → `abort_event.set()` → RTL
- [ ] 任務邏輯收到中止後立即停止送 goto（以 `MissionAborted` 跳出）
- [ ] 故障注入：正方形開始後 ~20 s 設 `SIM_BATT_VOLTAGE=10.5`（以非阻塞計時方式，例如在 `fly_to` 迴圈中檢查或 `threading.Timer`）
- [ ] 主流程：確認 RTL → 等落地上鎖 → exit 0
- [ ] 若正方形飛完仍未觸發 → exit 1
- [ ] 處理自駕儀自身 failsafe 已切 RTL/LAND 的情況
- [ ] 實機測試：日誌依序呈現完整故事
- [ ] 測試後 `docker compose restart`

## Phase 5 — 交付

- [ ] `NOTES.md`（英文，約半頁）：
  - [ ] 架構與並行模型選擇理由（見 [03_architecture.md](03_architecture.md) §2）
  - [ ] 就緒判斷、確認策略、重試/timeout 取值依據
  - [ ] 「約 20 秒」計時起點定義
  - [ ] 更多時間會做的事（例如：任務協定、更完整測試、連線斷線重連、型別檢查）
  - [ ] 意外發現（例如：ACK 與實際狀態不同步、auto-disarm、自駕儀自身電池 failsafe）
- [ ] 根目錄 `README.md` 不修改（或僅在最底部加「如何執行」段落）
- [ ] `pytest` 全部通過
- [ ] 清楚的 git commit 歷史（每個 Phase 至少一個 commit）

## Phase 6 — Bonus（可選，擇一）

- [ ] B-1：AUTO 任務上傳與飛行，或
- [ ] B-2：Geofence 監控（可複用 Part 3 的 listener 機制）

---

## 進度紀錄

| 日期 | 執行者 | 完成項目 | 備註 |
|---|---|---|---|
| 2026-10-04 | 規劃 AI | 建立 `docs/` 規格文件 | 尚未開始實作 |
