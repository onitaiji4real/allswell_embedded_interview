# Allswell Technology Services — 嵌入式軟體工程師（題目中文翻譯）

> 本文為根目錄 [`README.md`](../README.md) 的繁體中文翻譯。專有名詞（MAVLink、GUIDED、RTL、SITL 等）保留原文。

歡迎！在這個任務中，你要撰寫 Python 程式，透過 MAVLink 指揮一台模擬的四軸無人機——這和我們真實飛機使用的協定相同。模擬器是 ArduPilot SITL（與我們無人機上飛行的自動駕駛韌體相同，只是編譯成可在桌機上執行的版本），所以你在這裡寫的一切都能直接對應到真實的飛行程式碼。

**我們寧願看到兩個部分做得好，也不要三個部分做得草率。** 如果時間不夠，請在你的筆記中描述接下來會怎麼做。

---

## 環境設定（15 分鐘）

需求：Docker、Python 3.10+。支援 macOS（Intel 與 Apple Silicon）、Linux 與 Windows。

```bash
# 1. 啟動模擬器（第一次建置會編譯 ArduPilot —— 大約需要 10 分鐘）
docker compose up -d --build

# 2. 安裝 Python 相依套件
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# 3. 驗證：大約 20 秒內應該會看到即時遙測資料
python starter/telemetry.py
```

模擬無人機位於澳洲坎培拉附近的一處場地，並在 `tcp:127.0.0.1:5760` 監聽 MAVLink。任何時候要把無人機重置成乾淨狀態：`docker compose restart`。

提示：你也可以將地面站（例如 QGroundControl，或 Windows 上的 Mission Planner）連到 `tcp:127.0.0.1:5762`，在地圖上觀看你的程式如何駕駛無人機。非必要，但對除錯非常有幫助。

---

## 規則

- 使用 `pymavlink`（已提供）。請**不要**使用更高階的 SDK（DroneKit、MAVSDK）——我們想看你如何處理協定本身。
- 除此之外只能使用標準函式庫（若有寫測試，可加上 `pytest`）。
- **不可用盲目的** `time.sleep()` **取代確認。** 指令可能被拒絕、訊息可能遺失；你的程式應該透過觀察遙測資料與確認回應（acknowledgement）來確認結果，並設定合理的逾時（timeout）。
- 你可以擴充 `starter/drone.py`，或依喜好重新組織——骨架只是建議，不是硬性要求。

實用參考：

- MAVLink 訊息：<https://mavlink.io/en/messages/common.html>
- ArduCopter 飛行模式：GUIDED = 接受來自機載電腦（companion computer）的位置指令；RTL = 返航（Return To Launch）。
- 指令以 `COMMAND_LONG` 送出，並以 `COMMAND_ACK` 確認。

---

## Part 1 — 解鎖並起飛

撰寫 `part1_takeoff.py`：

1. 連線到無人機並等待它準備好可以飛行。引導式起飛需要位置估計，在**開機後約 40–60 秒**模擬的 GPS 與 EKF 才會收斂（提示：`EKF_STATUS_REPORT` 旗標，或 `STATUSTEXT` 訊息）。太早解鎖會被拒絕——而且已解鎖但沒起飛的無人機會在約 10 秒後自動上鎖（auto-disarm）。
2. 切換到 GUIDED 模式。確認模式真的切換成功。
3. 解鎖馬達（arm）。透過確認回應（ACK）**以及**遙測資料雙重確認。
4. 下達起飛指令，飛到 Home 上方 **15 公尺**。
5. 爬升過程中印出高度，當與目標高度相差 0.5 公尺以內時印出 `REACHED`。乾淨地結束程式。

每一個步驟都必須處理失敗（指令被拒、逾時）：印出清楚的錯誤並以非零狀態碼結束——不可以卡住，也不可以帶著 traceback 崩潰。

## Part 2 — 飛一個正方形

撰寫 `part2_square.py`（可重用 Part 1 的起飛程式）：

1. 起飛到 15 公尺。
2. 飛一個**每邊 80 公尺**的正方形，最後回到起始角點（提示：`SET_POSITION_TARGET_GLOBAL_INT`，或計算各角點座標後送出 goto 指令）。
3. 當**水平距離 2 公尺以內**時視為「抵達」該角點。飛行中以約 1 Hz 的頻率印出進度（到下一個角點的距離）。
4. 抵達最後一個角點後，下達 RTL 指令，並等待無人機降落且上鎖（disarm）。然後結束。

## Part 3 — 電池失效保護監控

真實任務會因真實故障而中止。撰寫 `part3_failsafe.py`：

1. 啟動與 Part 2 相同的正方形任務。
2. **同時（並行）**監控電池電壓（`SYS_STATUS`）。若電壓低於 **11.0 V**，立即中止任務：下達 RTL、記錄中止原因並附上時間戳記，並停止送出 goto 指令。
3. 為了讓這件事可被測試，你的腳本也要自己*製造*這個故障：在任務開始約 20 秒後，使用 MAVLink 參數協定把模擬器參數 `SIM_BATT_VOLTAGE` 設為 `10.5`，並確認參數真的被設定成功。
4. 預期輸出：任務開始 → 電壓下降 → 你的監控器在正方形飛到一半時抓到 → 無人機返航並降落 → 乾淨結束，且留下一份能「說故事」的日誌。

這題有趣的地方在於設計：任務邏輯與安全監控必須同時運作，並共用同一條 MAVLink 連線。執行緒（threads）、asyncio、或單一迴圈狀態機都可以接受——請告訴我們你為什麼選擇它。

## 加分題（僅在有剩餘時間時）

最多選一項：

- 使用 MAVLink 任務協定（mission protocol）將正方形上傳成正式的 AUTO 任務，並以 AUTO 模式飛行。
- 地理圍欄（geofence）監控：在 Home 周圍定義一個圓，接近時發出警告，越界時強制 RTL。

---

## 交付項目

1. 你的程式碼（`part1_takeoff.py`、`part2_square.py`、`part3_failsafe.py`，以及任何共用模組），以及其他用來確保功能正常的程式碼。
2. `NOTES.md` —— 約半頁：設計決策、如果有更多時間會怎麼改、任何讓你意外的事。

將整個資料夾以 git repo 連結的形式回傳。
