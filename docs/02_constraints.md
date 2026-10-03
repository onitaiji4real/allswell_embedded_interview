# 02 — 硬性限制（Constraints）

> 以下規則**不可違反**。違反任何一條等同交付失敗。

## 1. 相依套件

| 允許 | 禁止 |
|---|---|
| `pymavlink` | DroneKit、MAVSDK 或任何高階無人機 SDK |
| Python 標準函式庫（`threading`、`queue`、`asyncio`、`math`、`logging`、`argparse`、`dataclasses`、`time`、`sys`…） | `numpy`、`geopy` 等第三方套件 |
| `pytest`（僅用於測試） | 修改 `requirements.txt` 加入其他套件 |

- Python 版本：**3.10+**（可使用 `match`、`X | None` 型別語法）。

## 2. 不可盲睡（最重要）

- ❌ **禁止**：`arm(); time.sleep(5); takeoff()` —— 用時間猜測動作完成。
- ✅ **必須**：每個動作都以 **ACK + 遙測** 確認結果。
- ✅ 允許：在等待迴圈中使用 `recv_match(..., timeout=...)` 或極短的輪詢間隔（例如 `Event.wait(0.1)`）讓出 CPU；或用於控制輸出頻率（1 Hz 進度）/ 故障注入時機（20 秒）。這些不是「用 sleep 代替確認」。
- ✅ 所有等待都必須有 **timeout**，逾時要拋出明確錯誤。

## 3. 失敗處理

- 指令被拒（`COMMAND_ACK.result != MAV_RESULT_ACCEPTED`）→ 錯誤。
- 注意 `MAV_RESULT_IN_PROGRESS`（5）不是失敗，應繼續等待最終結果。
- 任何錯誤：印出清楚訊息 + **非零 exit code**，**不可**出現未捕捉的 traceback，**不可**卡住。
- 腳本入口建議統一模式：

```python
def main() -> int:
    try:
        ...
        return 0
    except DroneError as e:
        log.error("ERROR: %s", e)
        return 1
    except KeyboardInterrupt:
        log.error("Interrupted by user")
        return 130

if __name__ == "__main__":
    sys.exit(main())
```

## 4. 確認方式（雙重確認）

| 動作 | ACK | 遙測確認 |
|---|---|---|
| 切模式 | `COMMAND_ACK`（cmd 176） | `HEARTBEAT.custom_mode` |
| 解鎖 | `COMMAND_ACK`（cmd 400） | `HEARTBEAT.base_mode & MAV_MODE_FLAG_SAFETY_ARMED` |
| 起飛 | `COMMAND_ACK`（cmd 22） | `GLOBAL_POSITION_INT.relative_alt` 上升並到達 |
| 前往角點 | （`SET_POSITION_TARGET_GLOBAL_INT` 無 ACK） | `GLOBAL_POSITION_INT` 水平距離 ≤ 2 m；可輔以 `POSITION_TARGET_GLOBAL_INT` 回報確認目標已被接受 |
| 設參數 | `PARAM_VALUE` 回應 | 回應值 ≈ 設定值 |
| 降落 | — | `HEARTBEAT` 已上鎖（+ `EXTENDED_SYS_STATE`） |

## 5. 單一 MAVLink 連線

- Part 3 的任務邏輯與安全監控**必須共用同一條連線**（不可開第二條連線給監控器用）。
- `pymavlink` 的連線物件**不是執行緒安全**的：不可讓多個執行緒同時呼叫 `recv_match()`。架構方案見 [03_architecture.md](03_architecture.md)。

## 6. 環境

- SITL 連線：`tcp:127.0.0.1:5760`（程式）；`5762` 保留給地面站（QGroundControl）。
- 重置：`docker compose restart`（Part 3 執行後 `SIM_BATT_VOLTAGE` 會維持 10.5 V，**下次測試前務必重置**）。
- 不需修改 `sitl/Dockerfile` 與 `docker-compose.yml`。

## 7. 範疇控制

- 優先順序：Part 1 → Part 2 → Part 3 → `NOTES.md` → Bonus（最多一項）。
- **品質 > 數量**。時間不足時，把未完成部分寫進 `NOTES.md`。
- `starter/drone.py` 可自由擴充或重構。
