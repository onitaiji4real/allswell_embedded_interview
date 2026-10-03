# 06 — 手動測試指南（macOS）

先在下載或 clone 後的專案根目錄開啟終端機，再逐項執行。每個飛行測試都要等程式結束，並在下一項開始前重置 SITL；不要同時執行兩個飛行腳本。以下指令使用專案內的 `.venv/bin/python`，不需要啟用虛擬環境。

## 1. 準備環境

```bash
docker compose up -d --build
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
docker compose ps
```

首次 `docker compose up -d --build` 會編譯 ArduPilot，可能需要約 10 分鐘；之後通常只需 `docker compose up -d`。`docker compose ps` 應顯示 `sitl` 正在執行，並對外開放 TCP `5760`、`5762`。

### macOS 上重新連接 QGroundControl

在這台 Mac 上，每次 `docker compose restart` 或重新啟動 SITL 後，如果 QGroundControl 需要重新連線，**先**用遙測腳本確認模擬器正在送資料；此處不用再重啟一次 SITL：

```bash
.venv/bin/python starter/telemetry.py
```

若已先執行 `source .venv/bin/activate`，等價指令就是 `python starter/telemetry.py`。看到 `Heartbeat from system ...` 和持續更新的 `mode=... lat=... lon=... alt_rel=...` 後，按一次 **Ctrl+C** 結束遙測腳本，再於 QGroundControl 重新連接 TCP `127.0.0.1:5762`。`telemetry.py` 使用 `5760` 且會持續執行；開始飛行測試前，先確認它已結束。若手動停止時出現 `KeyboardInterrupt`，那是此檢查腳本收到 Ctrl+C，無需重置 SITL。

## 2. 單元測試（不需 SITL）

```bash
.venv/bin/python -m pytest -q
```

預期：`27 passed`。若未通過，先記錄失敗的測試名稱與錯誤訊息。

## 3. 正常飛行流程

每一項下方的 `echo` 必須緊接腳本執行，才能看到該腳本的退出碼。測試時可以讓 QGroundControl 連在 `5762` 觀察地圖；飛行程式使用 `5760`。

### Part 1：起飛

```bash
docker compose restart
.venv/bin/python part1_takeoff.py
echo "Part 1 exit=$?"
```

預期：等待 GPS／EKF 就緒，顯示 `Mode -> GUIDED`、`Armed`、逐秒高度、`REACHED`（15 m ± 0.5 m）；退出碼為 **0**。Part 1 結束時無人機仍在空中，下一項前務必重置 SITL。

### Part 2：80 m 正方形與降落

```bash
docker compose restart
.venv/bin/python part2_square.py
echo "Part 2 exit=$?"
```

預期：飛行 B、C、D、A 四個角點，每段顯示剩餘距離；之後確認 RTL，顯示 `Landed and disarmed`；退出碼為 **0**。

### Part 3：低電壓失效保護

```bash
docker compose restart
.venv/bin/python part3_failsafe.py
echo "Part 3 exit=$?"
```

預期：正方形開始約 20 秒後，日誌確認 `SIM_BATT_VOLTAGE=10.50`；監控器記錄 `ABORT: battery 10.50 V < 11.00 V`，停止後續導航、確認 RTL，落地上鎖；退出碼為 **0**。注入後的模擬電壓會保留到重置，因此下一項前必須重置 SITL。

### Bonus B-2：Home 地理圍欄

```bash
docker compose restart
.venv/bin/python bonus_geofence.py
echo "Bonus exit=$?"
```

預期：`Geofence initialized` 顯示飛控回報的 Home；距離約 80 m 時出現 `GEOFENCE WARNING`，達 100 m 時出現 `GEOFENCE BREACH` 並要求 RTL；落地上鎖，退出碼為 **0**。

## 4. 負向測試

### 飛行中 Ctrl+C

```bash
docker compose restart
.venv/bin/python part2_square.py
```

看到 `Leg 1/4` 或 `Leg 2/4` 後，按一次 **Ctrl+C**，讓程式完成安全返航；不要在等待降落期間再次中斷。結束後執行：

```bash
echo "Interrupted flight exit=$?"
```

預期：先看到 `Flight interrupted while armed`、`Mode -> RTL`，再看到 `Landed and disarmed`；退出碼為 **130**。

### 無效參數

```bash
.venv/bin/python part3_failsafe.py --fault-delay -1
echo "Invalid argument exit=$?"
```

預期：清楚提示 `--fault-delay must be a finite non-negative number`，退出碼為 **2**，無須啟動飛行。

### SITL 未啟動

此項會暫停模擬器及 QGroundControl 連線，建議最後再測。

```bash
docker compose stop sitl
.venv/bin/python part1_takeoff.py
echo "No SITL exit=$?"
docker compose up -d sitl
```

預期：顯示連線／心跳錯誤，退出碼為 **1**，沒有 Python traceback。重新啟動後若 QGroundControl 未連上，依第 1 節的 macOS 步驟先執行 `starter/telemetry.py`，再重新連接 `5762`。

## 5. 結束與查核

```bash
docker compose restart
git diff --check
git status -sb
git log --oneline -4
```

最後重置可清除飛行狀態與 Part 3 注入的電池電壓。`git status -sb` 用於確認有哪些尚未提交的本地修改；它不會推送到遠端。
