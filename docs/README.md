# 文件索引（AI 工程師交接入口）

> 本資料夾是給**後續接手的 AI 工程師**閱讀的規格與規劃文件。
> 原始需求來源：專案根目錄 [`README.md`](../README.md)（英文）。若本文件與原始 README 衝突，**以原始 README 為準**。

## 專案一句話說明

使用 `pymavlink` 透過 MAVLink 協定控制 ArduPilot SITL 模擬四軸無人機：起飛 → 飛正方形 → 電池失效保護（failsafe）監控，並交付 `NOTES.md`。

## 建議閱讀順序

| 順序 | 文件 | 內容 |
|---|---|---|
| 1 | [00_README_zh-TW.md](00_README_zh-TW.md) | 原始 README 的完整繁體中文翻譯 |
| 2 | [01_requirements.md](01_requirements.md) | 每個 Part 的需求拆解與**驗收標準** |
| 3 | [02_constraints.md](02_constraints.md) | 不可違反的硬性限制（規則） |
| 4 | [03_architecture.md](03_architecture.md) | 建議的程式架構、檔案結構、並行設計 |
| 5 | [04_mavlink_reference.md](04_mavlink_reference.md) | MAVLink 訊息/指令速查與已知陷阱 |
| 6 | [05_task_plan.md](05_task_plan.md) | 分階段執行計畫與勾選清單 |

## 專案現況（2026-10-04 審查與修正後）

| 檔案 | 狀態 |
|---|---|
| `README.md` | 原始題目（英文） |
| `docker-compose.yml` | 啟動 SITL，對外開 `5760`（程式用）、`5762`（地面站用） |
| `sitl/Dockerfile` | 編譯 ArduCopter `Copter-4.5.7`，Home 在澳洲坎培拉 CMAC 場地，`-w` 每次啟動清除 EEPROM |
| `requirements.txt` | `pymavlink>=2.4.40`、`pytest>=8.0` |
| `starter/link.py`、`starter/drone.py`、`starter/mission.py`、`starter/geo.py` | 已實作單一接收執行緒、飛行控制、共用任務與座標計算 |
| `starter/telemetry.py` | 環境驗證腳本（印出模式、是否解鎖、經緯度、相對高度） |
| `part1_takeoff.py` / `part2_square.py` / `part3_failsafe.py` | 已實作；本次審查前的正常路徑均在 SITL 跑通，修正後驗證見 [05_task_plan.md](05_task_plan.md) |
| `bonus_geofence.py` | 已實作 B-2 地理圍欄（選做） |
| `NOTES.md` | 已撰寫精簡英文交付筆記；提交前須納入 git commit |

## 給接手 AI 的核心原則

1. **先做好 Part 1、Part 2，再做 Part 3**。題目明說「兩題做好勝過三題草率」。Bonus 只在全部完成後才考慮，且最多一項。
2. **禁止用 `time.sleep()` 代替確認**。每個動作都要用 ACK + 遙測（telemetry）確認，並設合理 timeout。
3. **任何失敗都要印出清楚錯誤並以非零狀態碼結束**，不可卡住、不可噴 traceback。
4. 只能用 `pymavlink` + Python 標準函式庫（測試可用 `pytest`）。
5. 每完成一個階段，更新 [05_task_plan.md](05_task_plan.md) 的勾選狀態。
