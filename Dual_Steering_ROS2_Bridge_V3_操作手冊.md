# Dual Steering Motor — ROS 2 Bridge V3 操作手冊

> **系統版本**：ROS 2 Humble / STM32F407 / CANopen / BLVD-KRD / `dual_steering_ros2_bridge_V3.py`  
> **適用範圍**：兩顆轉向馬達的實驗台架控制；不包含行走輪馬達。  
> **重要安全提醒**：`all;stop` 是 CANopen Quick Stop 請求，**不是硬體緊急停止**。操作前必須固定機構、確認旋轉空間，並具備獨立硬體斷電／緊急停止方式。

## 1. 系統介紹

ROS 2 將角度／查詢指令發布到 `/motor/command`；Python Bridge V3 透過 USB-UART（115200 baud）與 STM32F407 通訊；STM32 再透過 CAN1（500 kbit/s）對兩顆 BLVD-KRD 發送 CANopen / CiA402 指令。

```text
ROS 2 Humble (NUC)
      │ /motor/command
      ▼
dual_steering_ros2_bridge_V3.py
      │ USB-UART, 115200 baud
      ▼
STM32F407（StartupSafe Firmware）
      │ CAN1, 500 kbit/s
      ├── Node ID 1 → M1：左轉向馬達
      └── Node ID 2 → M2：右轉向馬達
```

| 目標 | 說明 |
|---|---|
| `m1` | 左轉向馬達，CANopen Node ID 1 |
| `m2` | 右轉向馬達，CANopen Node ID 2 |
| `all` | 對 M1、M2 依序執行；**不保證精確同步** |

目前 Bridge 支援絕對與相對角度、位置讀取、暫時零點、雙馬達致能、狀態查詢與 Quick Stop。`sethome` 僅把當前位置設為 **STM32 RAM 中的暫時零點**，不是自動尋找真實機械零點；STM32 重新啟動／重新燒錄後必須重新建立。

## 2. 啟動 Bridge V3

先確認 STM32、兩顆 BLVD-KRD、CAN 與 USB-UART 已正確連接，且舊 Bridge 已停止（不可有兩個程式同時開啟同一個 UART Port）。

**Terminal 1 — 啟動 Bridge**

```bash
source /opt/ros/humble/setup.bash
python3 ~/Dual_Steering_Motor_Test/src/dual_steering_motor_bridge/dual_steering_ros2_bridge_V3.py
```

正常會看到類似 `Connected: /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0`、`Motor Command V1 ready: /motor/command`；這裡的 `Motor Command V1` 是程式內部訊息文字，不代表啟動的是舊 V1 檔案。

若已在執行，先在 Terminal 1 按 `Ctrl+C`，再重新執行上述指令。**僅重啟 Python Bridge 通常不會清除 STM32 的 RAM 零點；STM32 重新啟動則會。** 不要在馬達運動中重啟 Bridge。

## 3. 兩個關鍵監控 Topic

另開兩個 Terminal，並在每個 Terminal 先執行 `source /opt/ros/humble/setup.bash`。

**Terminal 2 — 指令處理結果 `/motor/result`**

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /motor/result
```

用來確認 Bridge 是否排入／接受指令，或回報錯誤，例如：

```text
QUEUED ENABLE motors=[1, 2]
OK M1 ENABLE ACKNOWLEDGED (brake state not independently sensed)
OK M2 ENABLE ACKNOWLEDGED (brake state not independently sensed)
OK ENABLE VERIFIED for motors=[1, 2] (CiA402 status only; physical brake unverified)
```

**Terminal 3 — STM32 原始狀態 `/steering/status`**

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /steering/status
```

用來確認 `STATUS`、`STATUSWORD`、`REF_VALID`、`MOVING`、`ACCEPT`、`REACHED`，例如：

```text
STATUS CAN=1 M1_READY=0 M2_READY=0 M1_MOVING=0 M2_MOVING=0 SAFE=1
OK M1 POS ACT_POS=1024 REL_STEPS=0 REF_VALID=1 MOVING=0
OK M2 POS ACT_POS=802917 REL_STEPS=0 REF_VALID=1 MOVING=0
```

**判讀重點**：

| 欄位 | 意義 |
|---|---|
| `CAN=1` | STM32 內部認為 CAN Bus 已啟動 |
| `SAFE=1` | StartupSafe 軟體狀態檢查通過；**不代表安全認證或物理煞車已驗證** |
| `REF_VALID=1` | 已在 STM32 RAM 設定暫時零點 |
| `M1_READY=1` / `M2_READY=1` | STM32 將對應馬達標記為已致能 |
| `MOVING=0` | STM32 沒有標記該馬達正在運動；仍應確認機械已實際停止 |
| `STATUSWORD` | BLVD-KRD 的 CiA402 狀態。`(STATUSWORD & 0x006F)==0x0027` 為 Operation Enabled |

## 4. 每次使用前的正確順序（必讀）

1. **確保馬達固定、可獨立安全停止、機構確實停止。** 上電後先不要直接下達角度命令。
2. 啟動 **Bridge V3**，並打開 `/motor/result` 和 `/steering/status` 兩個監控視窗。
3. 發送 `all;status` 和 `all;position`：檢查 `CAN=1`、`SAFE=1`、兩顆 `MOVING=0`，以及各自的 `REF_VALID`。
4. **僅當 `REF_VALID=0` 時才需重新設零點**：如果輪子已對準希望定義的正前方，且兩顆驅動器處於未致能／安全停止狀態，發送 `all;sethome`。確認兩顆都回覆 `OK M# ZERO`，之後 `REF_VALID=1`。
5. 發送 `all;enable`，確認 `/motor/result` 有 `OK ENABLE VERIFIED for motors=[1, 2]`，再用 `all;status` 核對 `M1_READY=1`、`M2_READY=1`。
6. 先用已驗證的小角度與低速度測試，例如 `all;A[1,-1];S50` 或 `all;A[10,-10];S100`；觀察 `ACCEPT`、`REACHED`、實際位置及 `MOVING`。
7. 完成後回到指定角度（例如 `all;home;S100`），等待兩顆實際停止。需要停止時使用 `all;stop`，但硬體急停必須獨立可用。

**重要：`all;enable` 之前必須已建立零點。** 若 `REF_VALID=0`，會出現 `ERR INIT: ZERO + HEARTBEAT REQUIRED`。如果已經設定零點、且 STM32 沒重新啟動，通常不必再執行 `sethome`。

### 輪子開機時不是正前方，怎麼辦？

不能直接把歪斜角度當成正式機械零點。台架調整時可以：先建立**臨時零點** → `all;enable` → 低速小角度調整至真實正前方 → 確認實際停止並使驅動器處於允許 `ZERO` 的未致能狀態 → 再 `all;sethome` 建立**正式零點** → 重新 `all;enable`。

**注意**：目前 `sethome` 在 `motor_ready=1` 時會被拒絕；`all;stop` 只請求 Quick Stop，不能單憑其回覆認定已完成安全失能。務必核對實際驅動器 CiA402 狀態、機構位置與煞車保持狀態，再重新建立零點。長期建議使用絕對角度編碼器或原點感測器。

## 5. Command 指令表

**發送格式**：

```bash
ros2 topic pub --once /motor/command std_msgs/msg/String "{data: 'all;status'}"
```

指令本體格式：`目標;動作[;速度]`。只有 `A`／`a` 區分大小寫：`A` = 絕對目標角度；`a` = 從當前實際角度增量移動。

| Command | 功能 |
|---|---|
| `all;status` | 查詢 STM32 狀態並讀取兩顆驅動器 Statusword |
| `all;position` | 查詢兩顆實際位置、相對 Steps、零點有效性與運動標記 |
| `m1;position`、`m2;position` | 分別查詢單顆馬達位置 |
| `all;probe` | 確認 M1、M2 CANopen 節點在線 |
| `all;sethome` | 將兩顆當前位置設為暫時零點（限允許的未致能狀態） |
| `m1;sethome`、`m2;sethome` | 單獨設定一顆暫時零點 |
| `all;enable` | 依序 M1 → M2 致能，並驗證 CiA402 Operation Enabled |
| `m1;enable`、`m2;enable` | 單顆致能 |
| `all;A[10,-10];S100` | M1 轉到 +10°，M2 轉到 −10°，設定速度 100 |
| `all;A[10,-10];S[100,200]` | 兩顆指定不同速度 |
| `m1;A0;S100` | M1 回到目前設定的零點 |
| `all;A0;S100` | 兩顆均移動至相對零點 |
| `m1;a10;S100` | M1 從當前位置再增加 10° |
| `all;a[10,-10];S100` | M1 從當前再 +10°，M2 再 −10° |
| `all;home;S100` | 兩顆返回暫時零點（等價目標 `A0`） |
| `all;stop` | 對兩顆要求 Quick Stop；**不是硬體急停** |

**限制**：目前只支援整數角度；絕對角度目標範圍為 −180°～+180°。V3 Bridge 的 `MAX_RPM=1000`，允許 `S1`～`S1000`，預設 `S100`。**STM32 必須也將 `MAX_TEST_SPEED_RPM` 設為 1000 並燒錄**，否則高於 300 的速度可能被拒絕。`S1000` 是寫入 CANopen `0x6081` 的設定值，物理單位尚待校正，不能直接認定為輸出軸 1000 RPM。依目前韌體假設，`20000 steps ≈ 輸出軸 1°`，實際輸出角度仍須外部量測驗證。

**目前不支援**：`m1;stop`（僅支援 `all;stop`）、`disable`、`brakeon`、`brakeoff`、安全連續 `jog`／`jogstop`。`all` 的雙馬達命令是**逐顆傳送**，非原子操作；若一顆成功另一顆逾時，可能發生部分執行。

## 6. 指令快速複製範例

**Terminal 4 — 發送命令**：

```bash
source /opt/ros/humble/setup.bash

# 1. 查詢
ros2 topic pub --once /motor/command std_msgs/msg/String "{data: 'all;status'}"
ros2 topic pub --once /motor/command std_msgs/msg/String "{data: 'all;position'}"

# 2. 僅當零點無效、且機構已正確對正與安全失能時執行
ros2 topic pub --once /motor/command std_msgs/msg/String "{data: 'all;sethome'}"

# 3. 確認兩顆 REF_VALID=1 後致能
ros2 topic pub --once /motor/command std_msgs/msg/String "{data: 'all;enable'}"

# 4. 小角度測試（先確認狀態、安全距離與停止手段）
ros2 topic pub --once /motor/command std_msgs/msg/String "{data: 'all;A[10,-10];S100'}"

# 5. 回到零點
ros2 topic pub --once /motor/command std_msgs/msg/String "{data: 'all;home;S100'}"

# 6. 請求兩顆 Quick Stop
ros2 topic pub --once /motor/command std_msgs/msg/String "{data: 'all;stop'}"
```

可選：在 Bash 建立簡寫函式（只在目前 Terminal 生效）：

```bash
motorcmd() {
  ros2 topic pub --once /motor/command std_msgs/msg/String "{data: '$1'}"
}

motorcmd 'all;status'
motorcmd 'all;position'
motorcmd 'all;sethome'
motorcmd 'all;enable'
motorcmd 'all;A[10,-10];S100'
motorcmd 'all;stop'
```

## 7. 常見錯誤與處理

| 訊息／現象 | 可能原因 | 建議處理 |
|---|---|---|
| `ERR INIT: ZERO + HEARTBEAT REQUIRED` | `REF_VALID=0`、HB 逾時、CAN 不可用或仍在運動 | 先查 `all;position`、`all;status`；若零點無效，確認機械對正後再設零點 |
| `REF_VALID=0` | STM32 重新啟動／燒錄，RAM 零點失效 | 不要直接 Enable；先重新確認實體零點與 `sethome` |
| `ERR FORMAT use M1: or M2:` | STM32 收到格式異常的 UART 資料 | 檢查 UART IRQ、Ring Buffer、線路及指令來源，不要盲目重送運動 |
| `ERR UART RESPONSE TIMEOUT: M2:A...` | Python 未如期收到回覆；**馬達可能已執行** | 不要立即重發；先用硬體手段確保停止，再查位置與驅動器狀態 |
| `REJECTED: ENABLE/ZERO/BUSY/LIMIT` | 尚未致能／零點無效／仍在運動／超出限位或速度限制 | 檢查 READY、REF_VALID、MOVING、角度及速度設定 |
| `READY=0` 但 Statusword 顯示 Operation Enabled | MCU 內部旗標與驅動器實際狀態不一致 | 禁止繼續運動；確認安全失能並排查狀態同步 |

## 8. 其他 ROS 2 Topics 與注意事項

| Topic | 型別 | 用途 |
|---|---|---|
| `/motor/command` | `std_msgs/msg/String` | 發送分號格式的 Command |
| `/motor/result` | `std_msgs/msg/String` | Bridge 佇列、確認、錯誤回報 |
| `/steering/status` | `std_msgs/msg/String` | STM32 原始回覆、位置與狀態 |
| `/left_steering/feedback_deg` | `std_msgs/msg/Float64` | M1 相對角度回授 |
| `/right_steering/feedback_deg` | `std_msgs/msg/Float64` | M2 相對角度回授 |
| `/left_steering/command_deg` | `std_msgs/msg/Float64` | 舊有 M1 角度命令介面 |
| `/right_steering/command_deg` | `std_msgs/msg/Float64` | 舊有 M2 角度命令介面 |
| `/steering/command` | `std_msgs/msg/Float64MultiArray` | 舊有雙馬達角度命令介面 |
| `/steering/control` | `std_msgs/msg/String` | 舊有控制介面 |

`/motor/result` 的 `OK COMMAND COMPLETED (accepted; motion may still be running)` 只代表 Bridge 本次指令流程已完成，**不代表機械運動已到位**。應觀察 `/steering/status` 中的 `REACHED`、`MOVING=0` 與實際位置，並在重要動作時做外部機械確認。`OK ENABLE VERIFIED` 只驗證 CiA402 狀態，**不獨立驗證實體磁煞車是否釋放**。

---

**已驗證的台架操作**：兩顆可完成 `sethome → enable → A[10,-10] → A0` 的角度回授流程。**仍待驗證**：不同負載下的實際角度精度、速度單位、硬體停止能力、機械限位、線束扭轉與長時間可靠性。
