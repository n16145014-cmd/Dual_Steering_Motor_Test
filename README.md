# Four Steering Motors — ROS 2 控制系統操作手冊

> **文件版本**：2026-11-08（依目前三個 Python 節點的程式碼與已提供的台架測試記錄整理）  
> **運行環境**：Ubuntu / ROS 2 Humble / 兩塊 STM32F407 / CANopen / 四台 BLVD-KRD / 四顆 BLMR6200SKM-GFV-F + GFV6G200  
> **適用範圍**：四顆**轉向**馬達的實驗台架；不包括四顆驅動輪轂馬達或自主轉向控制器。  
> **安全限制**：`all;stop` 只是要求 STM32 發送 **Quick Stop**，不是安全認證的 E-stop，也不能保證驅動器已失能。測試必須有獨立硬體停止方式、機械固定與淨空。

## 1. 目前使用的三節點架構

```text
ROS 2 Controller / Terminal / HMI
            │ /steering/four/command  (std_msgs/String)
            ▼
 dual_steering_ros2_bridge_V5.py
   /four_motor_router_v5
            │ 分派命令、轉發結果（不碰 UART）
      ┌─────┴─────────────┐
      ▼                   ▼
 bridge_sidea.py       bridge_sideb.py
 /bridge_side_a        /bridge_side_b
      │                   │
      │ USB-UART          │ USB-UART (115200 baud)
      ▼                   ▼
 STM32 #1              STM32 #2
      │ CAN1 500 kbps     │ CAN1 500 kbps
      ├ Node 1           ├ Node 3
      └ Node 2           └ Node 4
```

**重要**：`bridge_sideb.py` 直接匯入 `bridge_sidea.py` 的 `main`，因此兩側運行**相同的 `MotorBridge` 類別**；差異在 Side 名稱、串口、Node ID 配置。`bridge_sidea.py` 和 `bridge_sideb.py` 必須放在同一目錄。

### 1.1 各程式的職責

| 程式 | 職責 | 不負責 |
|---|---|---|
| `dual_steering_ros2_bridge_V5.py` | 解析四顆馬達命令、全域/本地編號映射、向兩側 Topic 分派、轉發結果與狀態 | Heartbeat、UART、CANopen、跨側同步、動作到位驗證 |
| `bridge_sidea.py` | 共用的 V4 類型 Bridge 核心，UART、Heartbeat、命令佇列、POS 讀取、Enable/Zero、CiA402 查詢 | 四顆跨側同步控制 |
| `bridge_sideb.py` | 用 Side B 參數啟動相同的 Bridge 核心 | 維護第二份獨立的控制邏輯 |
| STM32 韌體 | CANopen 指令、安全監控與暫時零點 | 經獨立安全認證的 E-stop |

不要同時啟動舊的 V3、V4、V5.1 或其他會佔用相同 UART／訂閱同一控制 Topic 的程式。

## 2. 馬達編號與交換配置

**預設配置**：

| V5 全域編號 | Side | Bridge 本地編號 | CANopen Node ID |
|---|---|---|---|
| `m1` | Side A | `m1` | Node 1 |
| `m2` | Side A | `m2` | Node 2 |
| `m3` | Side B | `m1` | Node 3 |
| `m4` | Side B | `m2` | Node 4 |

**Side B 透過 UART 發送的仍是 `M1:`、`M2:`**，不是 `M3:`、`M4:`；STM32 驅動器回覆中的實際 Node ID 則可為 3、4。V5 對外使用全域 `m1`～`m4`。

可以對調 Side A/Side B 所管理的 Node **組**（`1,2` 與 `3,4`），但三處必須一致：

1. Side A/Side B 的 `-p node_ids:=...`；
2. V5 的 `-p side_a_nodes:=...`、`-p side_b_nodes:=...`；
3. 實際連接的 STM32 韌體 CANopen Node ID 及實體接線。

單純改 ROS 2 參數**不會**改變 STM32 韌體中的 CANopen Node ID。

## 3. 啟動流程（預設 Node 1/2、3/4）

在馬達**已安全停止**時，先停止舊 Bridge。啟動前檢查串口：

```bash
ls -l /dev/serial/by-path/
```

以下路徑是目前測試電腦曾使用的範例，USB 插孔或拓樸更動後應重新確認，不要僅依 `ttyUSB0/1` 猜測：

**Terminal A — Side A**

```bash
source /opt/ros/humble/setup.bash
cd ~/Dual_Steering_Motor_Test/src/dual_steering_motor_bridge
python3 bridge_sidea.py --ros-args \
  -r __node:=bridge_side_a \
  -p node_ids:=1,2 \
  -p port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:8.2:1.0-port0
```

**Terminal B — Side B**

```bash
source /opt/ros/humble/setup.bash
cd ~/Dual_Steering_Motor_Test/src/dual_steering_motor_bridge
python3 bridge_sideb.py --ros-args \
  -r __node:=bridge_side_b \
  -p node_ids:=3,4 \
  -p port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:8.3:1.0-port0
```

**Terminal C — V5 Simple Router**

```bash
source /opt/ros/humble/setup.bash
cd ~/Dual_Steering_Motor_Test/src/dual_steering_motor_bridge
python3 dual_steering_ros2_bridge_V5.py
```

如果路由檔案已重新命名為 `V5.py`，將最後一行換成 `python3 V5.py`，勿同時運行兩個 Router。

**核對三個節點**：

```bash
ros2 node list
ros2 topic info /steering/side_a/motor/command -v
ros2 topic info /steering/side_b/motor/command -v
```

預期節點：

```text
/bridge_side_a
/bridge_side_b
/four_motor_router_v5
```

每側命令 Topic 預期一個 Router publisher、一個 Bridge subscriber。只檢查 Topic 連接不代表驅動器已 Enable。

### 3.1 Node 組交換範例

當實際 Side A 的 STM32 改為 Node 3、4，Side B 改為 Node 1、2，兩側 Bridge 的 `node_ids` 對調，V5 改用：

```bash
python3 dual_steering_ros2_bridge_V5.py --ros-args \
  -p side_a_nodes:=3,4 \
  -p side_b_nodes:=1,2
```

此時 V5 全域 `m3`、`m4` 會送到 Side A，`m1`、`m2` 會送到 Side B；**Node 編號不等於車輪在機器人上的幾何位置**。應另外記錄實際前左／前右／後左／後右對應關係。

## 4. ROS 2 Topics

| Topic | 型別 | 說明 |
|---|---|---|
| `/steering/four/command` | `std_msgs/msg/String` | 唯一建議的四馬達控制入口 |
| `/steering/four/result` | `std_msgs/msg/String` | V5 路由與兩側 V4 的結果 |
| `/steering/four/status` | `std_msgs/msg/String` | V5 加註 `side_a:` / `side_b:` 的 MCU 原始回覆 |
| `/steering/side_a/motor/command` | `std_msgs/msg/String` | Side A 的兩馬達控制命令 |
| `/steering/side_b/motor/command` | `std_msgs/msg/String` | Side B 的兩馬達控制命令 |
| `/steering/side_a/motor/result` | `std_msgs/msg/String` | Side A 命令處理結果 |
| `/steering/side_b/motor/result` | `std_msgs/msg/String` | Side B 命令處理結果 |
| `/steering/side_a/status` | `std_msgs/msg/String` | Side A MCU 原始回覆 |
| `/steering/side_b/status` | `std_msgs/msg/String` | Side B MCU 原始回覆 |
| `/steering/side_a/left_steering/feedback_deg` 等 | `std_msgs/msg/Float64` | **每側本地** M1/M2 的相對角度回授；並非四馬達統一順序陣列 |

共用 Bridge 還保留 Side-scoped legacy Topics（`/steering/<side>/control`、`/steering/<side>/command`、`/steering/<side>/left_steering/command_deg`、`/steering/<side>/right_steering/command_deg`）。日常測試請優先使用 `/steering/four/command`，避免多個命令來源互相干擾。

**監看輸出**（另開 Terminal）：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /steering/four/result
# 另一個 Terminal：
ros2 topic echo /steering/four/status
```

## 5. 建議台架操作順序

> 以下為受控台架測試程序。每次開始前確認實際輪叉機械方向、保護空間、供電電壓、電源限流狀態與**獨立硬體停機措施**。不要把字串 `SAFE=1` 視為安全認證。

1. 完成電源、線路、USB、CAN 終端與機構檢查；確保原先沒有尚未排除的 MCU/驅動器故障。
2. 啟動 Side A、Side B、V5。先只讀查詢 `all;status`、`all;position`，確認 CAN 及位置回覆正常。
3. 核對每個輪叉的**機械零位**。`REF_VALID=1` 只表示 STM32 中有暫時零點，不保證它是車輪正前方。
4. 如需 `sethome`，請確保**實體已完全停止、沒有待執行角度指令**；目前 Bridge 會送 `M1:ZERO/M2:ZERO`，但是否可在 Enabled 狀態設零須由**實際 STM32 韌體**決定。未確認韌體規則前，勿在 Enabled 或運動中強制設零。
5. 如未 Enable，且韌體要求的零點和 Heartbeat 等條件已滿足，執行 `all;enable`；確認兩側 `OK ENABLE VERIFIED`，並以 `all;status` 查 CiA402 狀態。
6. 先做**單馬達、小角度、低設定速度**測試，確認方向與實體到位；再測四馬達不同角度與低速聯合動作。
7. 查詢 `all;position`，確認四顆均 `REF_VALID=1`、`MOVING=0`、`REL_STEPS` 符合目標，並用外部角度工具核對機械轉角。

**不要假定每次啟動都需要重做 `sethome`**：只重啟 Python Bridge，一般不會清除 STM32 的 RAM 零點；STM32 復位／斷電後則需重新確認。增量編碼器不提供斷電後的絕對輪叉方向；即使系統將開機位置當作計數零點，也不等於機械正前方。

## 6. 常用四馬達指令

指令格式：`m1|m2|m3|m4|all;動作[;S速度]`。`A` = 以目前軟體零點為基準的**絕對目標**，`a` = **相對目前位置**；目前只支援整數度。

| 指令 | 用途 |
|---|---|
| `all;status` | 兩側 STM32 的 STATUS + 各馬達 PROBE |
| `all;position` | 四顆馬達位置查詢 |
| `all;probe` | 查詢四個 CANopen 節點 |
| `all;sethome` | 各馬達當前位置設為暫時零點（是否允許取決於韌體狀態） |
| `all;enable` | 兩側依序執行每側馬達 Enable，並驗證 CiA402 |
| `m3;A10;S100` | 控制全域 Motor 3 至 +10°，預設映射 Side B 本地 `m1` |
| `all;A[10,-10,10,-10];S100` | 四顆分別至 +10°、−10°、+10°、−10° |
| `all;A-90;S700` | 四顆至 −90°，相同速度設定（僅在機械行程允許的台架情況使用） |
| `all;a[1,-1,1,-1];S100` | 四顆相對移動指定角度 |
| `all;home;S100` | 四顆返回目前軟體零點 |
| `all;stop` | 向兩側請求 Quick Stop；**不是 E-stop** |

命令範例：

```bash
# 只讀查詢
ros2 topic pub --once /steering/four/command std_msgs/msg/String "{data: 'all;status'}"
ros2 topic pub --once /steering/four/command std_msgs/msg/String "{data: 'all;position'}"

# 只有在機械零點已確認、滿足 MCU ZERO 條件時才使用
ros2 topic pub --once /steering/four/command std_msgs/msg/String "{data: 'all;sethome'}"

# 需要致能時；請先確認韌體前置條件與機械安全
ros2 topic pub --once /steering/four/command std_msgs/msg/String "{data: 'all;enable'}"

# 運動指令（僅在全部檢查通過、獨立停機方式就緒時）
ros2 topic pub --once /steering/four/command std_msgs/msg/String "{data: 'm1;A1;S100'}"

# 軟體 Quick Stop 請求
ros2 topic pub --once /steering/four/command std_msgs/msg/String "{data: 'all;stop'}"
```

可用 Bash 函式減少輸入：

```bash
steercmd() {
  ros2 topic pub --once /steering/four/command std_msgs/msg/String "{data: '$1'}"
}
steercmd 'all;status'
steercmd 'all;position'
```

**V5 對全域 `all;A[...]` 的四個值依全域 `m1,m2,m3,m4` 排序**；它會依配置把兩顆一組送給各側 Bridge，並不保證兩側同時起轉。

## 7. 回覆判讀與實際到位

| 訊息 | 正確解讀 |
|---|---|
| `ROUTED side_a: ...` | V5 已發布到 Side A 的 ROS Topic，**不是完成動作** |
| `side_a: QUEUED ...` | Side A Bridge 已排入佇列 |
| `OK M# ACCEPT ANGLE=...` | STM32 接受了對應角度請求 |
| `OK COMMAND COMPLETED (accepted; motion may still be running)` | 該側 Bridge 指令交易完成，不代表實際角度到位 |
| `OK ENABLE VERIFIED ...` | 該側已依 CiA402 Statusword 驗證 Enable；不代表物理煞車已獨立驗證 |
| `REF_VALID=1` | RAM 軟體零點有效，不保證機械零點正確 |
| `MOVING=0` | MCU 回報未在運動；仍需確認實際機械狀態 |
| `STATUSWORD=0x0337` | 先前台架曾觀察到的 Operation Enabled 狀態範例；可用 `(word & 0x006F)==0x0027` 判定 CiA402 核心狀態 |

先前台架記錄已顯示：四顆馬達完成 Enable、接受 −90°→+90°→0°（S700）的指令，最後四顆 `REL_STEPS` 接近 0、`REF_VALID=1`、`MOVING=0`。**這是先前實驗觀察，不是每次開機後自動成立**；也尚未驗證實際輪叉角度、額定負載轉矩、完整同步與可靠性。

位置理論換算目前為 `20000 steps ≈ 1°`，但輸出軸實際角度、減速機間隙及安裝誤差仍要外部量測。`S1`～`S1000` 是韌體接受的**設定值**，不是已驗證的輪叉 RPM。

## 8. 已知問題與故障排查

| 訊息／狀況 | 可能原因、建議 |
|---|---|
| `ERR M3/M4 POS FAILED` | STM32/CANopen 位置讀取失敗；先檢查驅動器電源、CAN、狀態及 MCU 原始錯誤，不要立即重發運動 |
| `ERR UART RESPONSE TIMEOUT: M1/M2:POS` | V4 等待回覆超時；可能是真正沒有回覆，也可能是收到 `ERR ... POS FAILED` 卻未被舊版匹配邏輯結束 |
| `ERR POSITION_FEEDBACK QUICKSTOP BOTH` | MCU 位置回饋異常而要求停止；需查原始故障，不應直接忽略 |
| `ERR HOST_TIMEOUT QUICKSTOP BOTH` | STM32 未在規定時間收到 Host Heartbeat；可能與 UART 查詢逾時或其他阻塞有關，需查時間戳 |
| `ERR UART_RX_FAULT QUICKSTOP BOTH` | MCU UART 接收故障，停止運動並排查串口通訊 |
| `ERR INIT: ZERO + HEARTBEAT REQUIRED` | Firmware 拒絕 Enable；檢查 RAM 零點、Host HB、CAN、MOVING/其他實作前置條件；不能只根據字面認定單一原因 |
| `REF_VALID=0` | RAM 參考零點無效；重新確認真實輪叉方向後才設零 |
| Side A/B 回覆錯位 | 檢查 V5 `side_*_nodes`、Bridge `node_ids` 和 STM32 燒錄配置是否三者一致 |
| 電源進入 CC／電壓下跌 | 四台 BLVD-KRD 共用電源、瞬時電流可能達限流；停止高負載測試並記錄供電，不能單靠提高設定來掩蓋故障 |

### 8.1 POS FAILED 軟體問題（版本確認必讀）

**本次用來核對 README 的 `bridge_sidea.py` 原始碼，對 `position` / `poll` 的匹配仍只接受成功的 `OK ... POS`，沒有完成 `ERR M# POS FAILED` 匹配修正。** 所以不要把此版本視為已修好的 Bridge。如果部署了後續修補包，必須再檢查實際部署檔案與版本，而不是僅憑 ZIP 名稱判斷。

修正範圍應限於共用 Bridge 的回覆識別及失敗處理；V5 Router 不需要因此增加自動 STATUS 刷新。**修正回覆匹配不等於消除底層 CANopen POS FAILED。**

### 8.2 目前測試電源

先前記錄：GW Instek SPD-3606，單主通道設定約 24V，四台 BLVD-KRD 並聯共用；曾使用 2.15A 限流，後續提高至 3.0A 進行台架測試。3.0A 是**四台合計**的上限，不是每台的上限。SPD-3606 的 30V/6A 量程允許設置 3.0A，但並不保證可支援四馬達負載／峰值工況；應觀察 CV/CC 與驅動器端電壓，並核對線材、保護器件及設備額定值。

## 9. Set Home 與增量編碼器的重要限制

- 增量編碼器在失去供電後通常無法自己提供**機械絕對方向**；控制器重新起始計數不代表輪叉真的回到正前方。
- 本系統的 `sethome` 是**暫時軟體零點**，只在正確機械位置確認後才有意義。它不是 Homing Sensor，自動零點與物理零點不同。
- 你希望在 Enabled 且靜止時重設零點，**Bridge 的 `sethome` 命令路由本身沒有禁止**；但是否接受、會否產生意外轉動，須核對實際 STM32 `ZERO` 實作。**目前提供的三份 Python 檔沒有包含 MCU 韌體，不能確定 Enabled 狀態可否安全設零。**
- `all;stop` 是 Quick Stop，並不等於 Driver Disable；不能拿它當成「已安全失能，可以設零」的證明。
- 若要做正式農田自主轉向，建議另加輪叉絕對位置量測或原點感測與開機標定程序。

## 10. 目前完成與待完成

**已由先前台架 Log 驗證**：V5 Router 對兩側 Topic 分派、兩塊 STM32 的多馬達 Enable 交易、四顆馬達收到角度命令，四顆在最後返回軟體零點，並持續回報位置。

**仍待驗證／完善**：最新版三檔部署後的長時可靠性、POS FAILED 回覆修補的硬體復測、實體輪叉角度、零位持久性、S1000 時的位置回饋與供電、機械負載轉矩、機械極限、跨側故障聯鎖、四顆精確同步與硬體安全停機。

---

> **維護原則**：四馬達的控制入口保持 V5 Simple Router；`bridge_sidea.py` 維護唯一的底層 Bridge 控制實現；`bridge_sideb.py` 僅為入口。不要回到 V3/V4 混用，也不要為修復單一 UART 錯誤把 Router 再度改成另一套狀態控制器。
