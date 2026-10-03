# 05 — 執行計畫與任務清單

> 接手的 AI 工程師：請依序執行，每完成一項就把 `[ ]` 改成 `[x]`，並在最下方「進度紀錄」追加一行。
> 每個 Phase 結束都要**在 SITL 上實際跑過**（`docker compose restart` 後執行），不是只寫完程式。
>
> **排程原則：先讓它飛起來，再把它做漂亮。** 原題強調「兩題做好勝過三題草率」，所以每一題都是先求在 SITL 上跑通，測試、重構、額外功能一律往後排。標有【選做】的項目只在 Part 1–3 都跑通後才做。

---

## Phase 0 — 環境確認

- [x] `docker compose up -d --build`（首次約 10 分鐘）
- [x] 建立 venv 並 `pip install -r requirements.txt`
- [x] `python starter/telemetry.py` 在 ~20 s 內看到遙測
- [x] 【選做】QGroundControl 連 `tcp:127.0.0.1:5762` 觀察

## Phase 1 — Part 1 跑通（`part1_takeoff.py`）

> 只寫 Part 1 需要的最小底層，直接寫在 `starter/drone.py` 裡，不拆檔。

- [x] 新增 `starter/__init__.py`
- [x] `drone.py` 最小底層：
  - [x] 接收執行緒（唯一 `recv_match` 呼叫者）+ `TelemetrySnapshot`（lock 保護）
  - [x] HEARTBEAT 來源過濾（只認自駕儀）
  - [x] `send_lock` 保護所有送出
  - [x] `send_command_long()`：等對應 ACK、處理 IN_PROGRESS、拒絕時附上最近的 STATUSTEXT
  - [x] `wait_until(predicate, timeout, desc, abortable=False)`
  - [x] 接收執行緒在連線失效時要讓等待者得知（不可永久卡住）
- [x] `wait_ready_to_arm()`（EKF 旗標 + GPS fix，timeout 120 s，期間印狀態）
- [x] `set_mode("GUIDED")`（ACK + HEARTBEAT）
- [x] `arm()`（ACK + HEARTBEAT，被拒時在 timeout 內重試）
- [x] `takeoff(15)`（ACK，印高度，誤差 ≤0.5 m 時印 `REACHED`）
- [x] `main()` 統一錯誤處理、退出碼、日誌格式（見 [03_architecture.md](03_architecture.md) §5）
- [x] **SITL 實測**：正常路徑 exit 0
- [x] **SITL 實測**：SITL 未啟動 → 清楚錯誤 + exit 1（無 traceback）
- [x] **SITL 實測**：`docker compose restart` 後立即執行，確認程式會等待，而不是解鎖被拒後崩潰
- [x] git commit

## Phase 2 — Part 2 跑通（`part2_square.py`）

- [x] `starter/geo.py`：`offset_latlon`、`horizontal_distance_m`、`square_corners`
- [x] `tests/test_geo.py`（成本低，建議此時就寫）：80 m 偏移誤差 < 0.1 m、四邊長度 ≈ 80 m、距離對稱
- [x] `starter/mission.py`：把 Part 1 流程抽成 `takeoff_sequence(drone, 15)`
- [x] 以起飛後位置為 A 計算 A→B→C→D→A
- [x] `goto()`：在 `send_lock` 內檢查 `abort_event`（Part 3 需要，現在先寫好）
- [x] `fly_to()`：每 ~1 s 重送目標並印距離，≤ 2 m 視為抵達，每段有 timeout（`abortable=True`）
- [x] `set_mode("RTL")` + `wait_landed_disarmed()`（`abortable=False`）
- [x] **SITL 實測**：完整飛完並落地，exit 0
- [x] git commit

> ✅ 到這裡已經達成「兩題做好」。若時間緊迫，可以跳到 Phase 4 先寫 `NOTES.md`。

## Phase 3 — Part 3 跑通（`part3_failsafe.py`）

> 實作前**必讀** [03_architecture.md](03_architecture.md) §2 的規則 A（listener 不可等待）與規則 B（競爭條件）。

- [ ] `request_mode_nowait()`：只送不等的 DO_SET_MODE
- [ ] `set_param()`：PARAM_SET + PARAM_VALUE 確認 + 重試；**不受 `abort_event` 影響**
- [ ] 電池監控 listener（在接收執行緒中執行，**只做不會卡住的事**）：`voltage < 11.0 V` → 寫 log → `abort_event.set()` → `request_mode_nowait("RTL")`
- [ ] 任務邏輯收到中止後立即停止送 goto（`goto()` 拋 `MissionAborted` / `fly_to()` 跳出）
- [ ] 故障注入：正方形第一段開始後約 20 s 設 `SIM_BATT_VOLTAGE=10.5`；若在背景執行緒執行，結束前要 `join()`，確保「參數已確認」有寫進日誌
- [ ] 主執行緒捕捉 `MissionAborted` → 確認 RTL（若已是 RTL/LAND 就直接記錄）→ 等落地上鎖 → exit 0
- [ ] 參數確認失敗 → 即使已安全返航也 exit 1
- [ ] 若正方形飛完仍未觸發中止 → exit 1
- [ ] 處理自駕儀自身 failsafe 已切到 RTL/LAND 的情況
- [ ] **SITL 實測**：日誌完整呈現故事，且「參數已確認」和「ABORT」兩行都有出現
- [ ] **SITL 實測**：確認 ABORT 之後的日誌裡沒有任何 goto 送出
- [ ] 測試後 `docker compose restart`
- [ ] git commit

## Phase 4 — 交付

- [ ] `NOTES.md`（英文，**約半頁，精簡為上**；篇幅不夠時只挑重點，每項 1–2 句）：
  - [ ] 並行模型選擇理由（見 [03_architecture.md](03_architecture.md) §2）
  - [ ] 自行補充的解讀 I-1～I-3（見 [01_requirements.md](01_requirements.md) Part 3）：20 秒計時起點、中止返航 exit 0、未觸發 exit 1
  - [ ] 更多時間會做的事（挑 2–3 項）
  - [ ] 意外發現（挑 1–2 項，例如 ACK 和實際狀態不同步、auto-disarm、自駕儀自身電池 failsafe）
- [ ] 根目錄 `README.md` 不修改（或只在最底部加「如何執行」段落）
- [ ] `pytest` 全部通過
- [ ] 清楚的 git commit 歷史

## Phase 5 — 品質強化（全部【選做】，依價值排序）

- [ ] 【選做】`tests/test_drone_logic.py`：用假連線測 ACK 接受 / 拒絕 / 逾時路徑（不需 SITL）
- [ ] 【選做】Ctrl+C 時若在空中，嘗試下 RTL
- [ ] 【選做】1 Hz GCS 心跳
- [ ] 【選做】CLI 參數或環境變數覆寫連線字串
- [ ] 【選做】把 MAVLink I/O 從 `drone.py` 拆到 `link.py`
- [ ] 每項完成後都要重跑 Part 1–3 的 SITL 實測，確認沒有退步

## Phase 6 — Bonus（【選做】，擇一）

- [ ] B-1：AUTO 任務上傳與飛行，或
- [ ] B-2：Geofence 監控（可複用 Part 3 的 listener 機制，同樣要遵守規則 A）

---

## 進度紀錄

| 日期 | 執行者 | 完成項目 | 備註 |
|---|---|---|---|
| 2026-10-04 | 規劃 AI | 建立 `docs/` 規格文件 | 尚未開始實作 |
| 2026-10-04 | 規劃 AI | 依審查意見修訂 docs | 修正 listener 死鎖風險、中止和參數確認的競爭條件；改成先跑通再重構；統一日誌格式；把 I-1～I-3 列入 NOTES 必寫 |
| 2026-10-04 | 開發者 | 完成 Phase 0 與 Phase 1 | 已完成 Part 1 的底層架構、起飛測試並更新了相關的確認機制。 |
| 2026-10-04 | 開發者 | 完成 Phase 2 | 成功實作與測試 Part 2，飛行正方形並安全降落。 |
| 2026-10-04 | 開發者 | 完成 Phase 2 | 成功實作與測試 Part 2，飛行正方形並安全降落。 |
