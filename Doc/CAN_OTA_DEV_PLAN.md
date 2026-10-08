# CAN-OTA 开发计划 (Development Plan)

> 目标：把现有基于 Modbus RTU 的 OTA 升级流程，移植到 CAN 总线上，复刻达妙 (DM) 电机手册中的"寄存器写入" CAN 帧格式，作为 bootloader 的升级通道。**APP 与 bootloader 双向 CAN 联动**：APP 通过 CAN 接收"进 boot"命令复位进 bootloader，bootloader 通过 CAN 完成升级并跳回 APP。

---

## 0. APP↔boot CAN 联动（关键补充，对应用户疑问）

原计划只改 bootloader 是不够的——APP 必须也能通过 CAN 被"踢进 boot"，否则 CAN 升级只能等设备本来就停在 bootloader。

### 0.1 APP 侧新增 CAN 寄存器响应（`motor/User/can_slave.c`）
APP 原本 `can_slave_reg_frame()` 是空函数（0x7FF 广播寄存器帧无人处理）。现填充为 DM 帧格式：
- **读 CONTROL (RID 0x00)** → 返回 `7`(APP_RUNNING_FLAG)，表示"APP 在跑"。与 Modbus `0x0000` 读固定返回 7 对齐。
- **读 FW_SIZE (RID 0x01)** → 返回 `firmware_info_recv()` 的固件长度。
- **读 MD5_0..3 (RID 0x05..0x08)** → 返回 `firmware_info_recv()` 的 MD5（4×u32 小端）。
- **读 MODEL_0..3 (RID 0x0A..0x0D)** → 返回 16 字节 ASCII 电机型号/OTA 兼容 ID。
- **读 VERSION_0..3 (RID 0x30..0x33)** → 返回 16 字节 ASCII APP 固件版本。
- **写 CONTROL = 1 (ENTER_BOOT)** → `NVIC_SystemReset()`，复位进 bootloader。与 Modbus `0x0000` 写 1 复位完全等价。
- 其余 RID 不响应。

> APP 只做"读状态/读 MD5/读身份/复位进 boot"，**不做升级写 flash**——升级永远在 bootloader 里做（APP 跑业务时 flash 不应被写）。

### 0.2 CONTROL=7 运行标志语义（与 Modbus 对齐）
- APP 运行：读 CONTROL = `7`。
- bootloader 升级中：读 CONTROL = 当前命令字（如 `1`=ENTER 后）。
- bootloader 升级完成跳回 APP 后：读 CONTROL = `7`（host 据此确认 APP 活过来了）。
- 这套语义与 `modbus_ota.py` 的 `APP_RUNNING_FLAG=7` 一致，host 端逻辑可共用。

### 0.3 host 侧 `can_ota.py` 新增 `_ensure_bootloader()`
升级前先读 CONTROL：若 = `7`(APP 在跑)，则发"写 CONTROL=1"让 APP 复位进 boot，等设备重启；若 ≠7（已在 boot 或无响应），直接进升级流程。这样无论设备当前在 APP 还是 boot，`can_ota.py` 都能自动把设备导入升级状态。

---


## 1. 背景与约束

### 1.1 现状
- `motor_bootloader` 当前 OTA 通路 = Modbus RTU (USART3 @921600)：
  - `User/modbus_slave.c` 维护一张寄存器表（control / size_h / size_l / offset / data[64] / write_control），通过 `modbus_execute_ota_control_command()` 派发到 `ota_*()` 状态机。
  - `main.c` 主循环：`modbus_slave_recv()` → `modbus_slave_step()` → `ota_on_step()`。
- `ota` 组件 (`Components/ota/`) 是传输无关的纯状态机：WAIT→ENTER→PREPARE→BURN→CHECK，回调 `flash_erase/write`、`firmware_info_read/write`、`on_jump_app`。CAN-OTA 直接复用，不动它。
- `Hardware/can.c` 当前是"空壳"：只配了引脚/时钟/波特率/模式，**没有** RX FIFO、TX FIFO、全局滤波器、`MCAN_Start()`、send/recv 函数。无法收发。
- `Components/damiao/` 已存在并被 `eide.yml` 纳入编译，其 `damiao_on_recv()` 已把 `id==0x7FF` 的广播帧路由到 `on_reg(frame)` 回调——这是天然的寄存器写入入口。bootloader 目前没在 `main.c` 里用它。
- APP (`motor/`) 侧 `Hardware/can.c` 是完整版：32 元素 RX FIFO0 + 32 元素 TX FIFO + `MCAN_GlobalFilterConfig(NMF_ACCEPT_IN_RX_FIFO0,...)` + `MCAN_Start` + `can1_send/can1_recv`。**bootloader 直接照搬这套配置。**

### 1.2 DM 寄存器写入帧格式 (取自 DM_manual.txt L433-482)
所有寄存器访问帧 ID = `0x7FF`(STD，广播)，由 `D[0..1]=CANID_L/CANID_H` 寻址目标从机：

| 操作 | D[2] opcode | D[3] | D[4..7] | 响应 ID |
|---|---|---|---|---|
| 读寄存器 | `0x33` | RID | (无) | MST_ID |
| 写寄存器 | `0x55` | RID | 32-bit data (4B) | MST_ID |
| 保存到 flash | `0xAA` | `0x01` | (无) | MST_ID |

- 读响应：`ID=MST_ID STD, D[0..1]=CANID_L/H, D[2]=0x33, D[3]=RID, D[4..7]=数据`。
- 写响应：`ID=MST_ID STD, D[0..1]=CANID_L/H, D[2]=0x55, D[3]=RID`。
- `CANID_L/CANID_H` = 从机 11-bit CAN ID 的低/高字节。
- 用户明确：**只复刻帧格式（机制），不复刻 DM 的寄存器地址语义**（即 0x00=UV_Value 之类不要）。

### 1.3 核心设计矛盾
DM 写寄存器帧**每帧只带 4 字节数据**（D[4..7]），而 OTA 固件数据块 = **128 字节**（现有 Modbus 一次 `readwrite_registers` 传 64 寄存器）。必须把 128 字节块拆成多帧 DM 写、在 bootloader 端拼回 128 字节再交给 `ota_firmware_write()`。

---

## 2. 总体方案

### 2.1 帧定义（bootloader 私有寄存器 RID 表）
本设计**不复用 DM 的寄存器地址**，自定义一张 OTA RID 表（单字节 0x00..0x7F），每帧写 32-bit：

| RID | 名称 | 宽度 | 说明 |
|---|---|---|---|
| 0x00 | CONTROL | u32 | 命令字：1=ENTER,2=SET_SIZE,3=ERASE,4=WRITE_BLOCK,5=UPDATE_MD5,6=SAVE_JUMP,7=APP_RUNNING |
| 0x01 | FW_SIZE | u32 | 固件总长度（替代旧 size_h/size_l 两个 reg） |
| 0x02 | BLOCK_OFFSET | u32 | 当前 128B 块的索引（block_index） |
| 0x03 | SUB_OFFSET | u32 | 块内 4 字节子偏移 0..31（0→byte0,1→byte4,...31→byte124） |
| 0x04 | BLOCK_DATA | u32 | 块内 4 字节数据，按 SUB_OFFSET 写入 s_data_buf[SUB_OFFSET*4 .. +3] |
| 0x05 | MD5_0 | u32 | MD5 word0（读为主，写触发 UPDATE_MD5 时回读校验） |
| 0x06..0x08 | MD5_1..3 | u32 | MD5 word1..3 |
| 0x09 | STATUS | u32(RO) | 当前 OTA 状态/上一命令执行结果，供主机 read 0x33 轮询 |
| 0x0A..0x0D | MODEL_0..3 | u32 | 16 字节 ASCII 型号；BOOT 中写入候选型号，供 `CONTROL=8` 校验 |
| 0x30..0x33 | VERSION_0..3 | u32(RO) | 16 字节 ASCII 固件版本 |

> 注：MD5 16 字节 = 4 个 u32 word，host 写完后直接用 `ota_md5()` 比对，无需主机回读；但保留 read 能力便于调试。

### 2.2 写流程（主机 → bootloader）
一个 128 字节块的传输 = 32 次 `0x55` 写 `BLOCK_DATA(RID=0x04)`：
1. host 写 `BLOCK_OFFSET = i`（RID 0x02）。
2. host 循环 j=0..31：写 `SUB_OFFSET = j`（RID 0x03）→ 写 `BLOCK_DATA = u32`（RID 0x04，写入 `s_data_buf[j*4..j*4+3]`，小端）。
3. host 写 `CONTROL = 4(WRITE_BLOCK)` → bootloader 把 128B `s_data_buf` 交给 `ota_firmware_write(offset=i*128, data, 128)`。
4. host 读 `STATUS`(0x33, RID=0x09) 轮询是否成功（成功 = `OTA_CMD_WRITE_FIRMWARE_DATA`）。

> 优化（可选，见 §8）：把 SUB_OFFSET 隐含在写帧序号里、host 连续发 32 帧不读响应。第一阶段先做"每写一帧等响应"的稳健版本。

### 2.3 完整下载时序
1. host 解析 `.bin` 固定身份段，并读取设备 MODEL；型号不一致时在复位前终止。
2. 写 `CONTROL=1` → `ota_enter_check()`，状态 ENTER。
3. 写 MODEL_0..3 后写 `CONTROL=8`，BOOT 比较候选兼容 ID；失败时不允许进入下一步。
4. 写 `FW_SIZE=n`(RID 0x01) → 写 `CONTROL=2` → `ota_firmware_size(n)`，状态 PREPARE。
5. 写 `CONTROL=3` → `ota_firmware_erase()`，状态 BURN。
6. 对每个 128B 块执行 §2.2。
7. 写 `CONTROL=5` → `ota_md5_update()`，状态 CHECK；host 读 `MD5_0..3` 比对。
8. 写 `CONTROL=6` → BOOT 再次校验 APP Magic/型号后写元数据，回到 WAIT，超时后跳 APP。

### 2.5 固件身份与合法性校验

APP 镜像固定身份段位于绝对地址 `0x00011000`，对应 `.bin` 偏移 `0x1000`，布局为：

| 偏移 | 字段 | 长度 |
|---:|---|---:|
| `0x00` | Build Time | 8 字节 |
| `0x08` | Magic (`FID1`) | 4 字节 |
| `0x0C` | Motor Model / Compatibility ID | 16 字节 ASCII |
| `0x1C` | Firmware Version | 16 字节 ASCII |
| `0x2C` | Reserved | 4 字节 |

BOOT 自身身份段位于 F460JETA loader 固定地址 `0x0000F000`，版本格式为 `BOOT-Vx.x.x`；APP 版本格式为 `APP-Vx.x.x`。MD5 仍只负责字节完整性，型号兼容 ID 负责产品合法性。当前实现的是产品型号兼容 ID，不是每台实体电机的 OTP 唯一序列号绑定。

### 2.4 CAN ID 方案
- **从机 CAN ID**：bootloader 没有完整 `g_save`（loader 不跑业务 nvstore）。用编译期常量 `LOADER_CAN_ID_DEFAULT = 1`，并把 `LOADER_MASTER_ID_DEFAULT = 0`。
  - 后续若要支持多机/可配，再把 `can_id/master_id` 写入 `FIRMWARE_INFO` 页或独立 NV 区；第一阶段硬编码即可。
- 主机发：`ID=0x7FF`，`D[0]=LOADER_CAN_ID & 0xFF`，`D[1]=(LOADER_CAN_ID>>8)&0xFF`。
- bootloader 响应：`ID=LOADER_MASTER_ID`（=0），同样回 `D[0..1]=CANID_L/H`。

---

## 3. 文件改动清单

### 3.1 bootloader 改动

| 文件 | 动作 | 说明 |
|---|---|---|
| `Hardware/can.h` | **改** | 加 `can_msg_t` 结构 + `can_init`→`can1_init`/`can1_send`/`can1_recv` 声明（照搬 APP 版） |
| `Hardware/can.c` | **改** | 照搬 APP `motor/Hardware/can.c`：32 RX FIFO0 + 32 TX FIFO + `MCAN_GlobalFilterConfig(ACCEPT_IN_RX_FIFO0,...)` + `MCAN_Start` + send/recv |
| `User/can_slave.h` | **新建** | `can_slave_init()` / `can_slave_recv()` / `can_slave_step()` |
| `User/can_slave.c` | **新建** | DM 帧解析 + OTA 寄存器表 + `modbus_execute_ota_control_command` 的 CAN 等价物（见 §4） |
| `User/bsp.c` | **改** | `bsp_init` 里 `can_init()`→`can1_init()`；`bsp_deinit` 已有 `MCAN_DeInit`，保留 |
| `User/main.c` | **改** | 主循环加 CAN 路径（与 modbus 并存，见 §5） |
| `Components/damiao/*` | **不动** | 直接复用 |
| `Components/ota/*` | **不动** | 直接复用 |
| `User/modbus_slave.*` | **不动** | 保留 Modbus 通路（向下兼容） |
| `Doc/CAN_OTA_DEV_PLAN.md` | **新建** | 本文件 |

### 3.2 host 侧
| 文件 | 动作 | 说明 |
|---|---|---|
| `motor/can_ota.py` | **新建** | 对应 `modbus_ota.py` 的 CAN 版（见 §6），依赖 `python-can` |

> 放在 `motor/` 下与 `modbus_ota.py` 同级，便于共用 `build/firmware.bin`。

---

## 4. `User/can_slave.c` 设计要点

```
static struct damiao s_damiao;
static struct damiao_config s_damiao_cfg;

/* OTA 寄存器镜像（对应 §2.1 RID 表） */
static uint32_t s_reg_control;
static uint32_t s_reg_fw_size;
static uint32_t s_reg_block_offset;
static uint32_t s_reg_sub_offset;
static uint8_t  s_data_buf[128];        /* 128B 块缓冲 */
static uint32_t s_reg_status;

/* 回调：damiao_on_recv 命中 0x7FF 时调用 */
static void can_slave_on_reg(const struct damiao_frame *f);
static void can_slave_on_send(const struct damiao_frame *f);  /* -> can1_send */
```

`can_slave_on_reg(f)` 逻辑：
1. 校验 `f->dlc>=4` 且 `f->data[0..1]` == 本机 CANID_L/H。不匹配则丢弃（广播帧但不是发给我的）。
2. `op = f->data[2]`，`rid = f->data[3]`。
3. `op==0x55`(写)：取 `u32 val = LE(f->data[4..7])`，按 rid 写入对应 `s_reg_*` / `s_data_buf`；对 `BLOCK_DATA` 用 `s_reg_sub_offset` 定位；写完若 rid==CONTROL 且 val==WRITE_BLOCK，调用 `ota_firmware_write()`。**每收一帧即回 ACK**（`0x55` 响应帧）。
4. `op==0x33`(读)：按 rid 取值，组 `0x33` 响应帧（D[4..7]=val，大端或小端见 §7）回 `MST_ID`。
5. `op==0xAA` 且 `f->data[3]==0x01`：本 OTA 无需 save-to-flash（元数据由 `ota_jump_app` 内部 `firmware_info_write` 完成），回 ACK 即可，不调用任何 flash 保存。

`can_slave_recv()`：`while(can1_recv(&msg))` → 转 `damiao_frame` → `damiao_on_recv(&s_damiao, &frame)`。
`can_slave_step()`：目前无 1ms 定时任务（damiao 反馈在 bootloader 不需要），保留空实现或仅做超时看门。主循环里直接调 `can_slave_recv()`（事件驱动）。

`can_slave_init()`：填 `damiao_config`（id/master_id 用 §2.4 常量；`on_reg=can_slave_on_reg`；`on_send=can_slave_on_send`；其余 callback 置 NULL），`damiao_init()`。

---

## 5. `main.c` 主循环改动

```c
bsp_init();
ota_init(&g_loader_ota, &s_loader_ota_config, LOADER_OTA_STARTUP_TIMEOUT);
modbus_slave_init();
can_slave_init();              /* 新增 */

while (1) {
    modbus_slave_recv();
    can_slave_recv();          /* 新增：事件驱动收 CAN */
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0UL) {
        modbus_slave_step();
        can_slave_step();      /* 新增 */
        ota_on_step(&g_loader_ota);
    }
}
```
两条通路并存：主机用 Modbus 或用 CAN 都能升级。`ota_t` 是单实例，两条通路都会驱动同一个状态机——这是可接受的（同一时刻只用一种）。

---

## 6. `motor/can_ota.py` 设计要点

- 类 `CanOta`，接口对齐 `ModbusOta.download(id, data, timeout)` / `progress()` / `result()`。
- 依赖 `python-can`（`can.interface.Bus(interface='canalystii',...)` 或 `vector`/`socketcan`，按用户硬件选；第一阶段用 `serialbus`/`pcan` 占位，命令行 `--interface/--channel`）。
- 帧构造函数：
  - `_write_reg(rid, val)`：发 `ID=0x7FF, data=[canid_lo, canid_hi, 0x55, rid, val_b0..val_b3]`，等响应帧（`ID=master_id, D[2]==0x55, D[3]==rid`）超时重试。
  - `_read_reg(rid)`：发 `0x33`，解析响应 `D[4..7]`。
- `download()` 复刻 `modbus_ota.py` 的 md5 预检→enter→size→erase→write(128B 块 = 32 次 `_write_reg(BLOCK_DATA,...)`)→md5_update→compare→jump。
- 进度条/结果 enum 与 `modbus_ota.py` 一致，CLI 同构（`--firmware/--id/--timeout`，加 `--interface/--channel/--bitrate`）。

---

## 7. 字节序约定
- CAN 帧内 32-bit 数据 `D[4..7]`：**小端**（D[4]=LSB），与 `damiao.c` 里 `__damiao_read_float` 的 `memcpy` 习惯一致，也便于 `memcpy(&val, &f->data[4], 4)`。
- MD5 word：`s_data_buf` 拼成 128B 后整体送 `ota_firmware_write`，字节序与原 Modbus（高字节先）解耦——只要 host 和 bootloader 约定一致即可。本设计 host 端 `struct.pack('<I', val)` 小端。

---

## 8. 阶段划分（建议实施顺序）

**阶段 A — bootloader CAN 收发打通**
1. 改 `Hardware/can.h` / `can.c` 照搬 APP 版（send/recv + FIFO + GlobalFilter + Start）。
2. 改 `bsp.c` 调 `can1_init()`。
3. 临时在 `main.c` 自环或回显测试 `can1_send/recv`（可选，无硬件可跳过）。

**阶段 B — can_slave 解析 + OTA 接线**
4. 新建 `User/can_slave.h` / `can_slave.c`，实现 §4 的 `on_reg` 读写分发。
5. `main.c` 接入 `can_slave_init/recv/step`。
6. 静态走查：CONTROL=1→2→3→4(32×BLOCK_DATA)→5→6 的状态流转。

**阶段 C — host 脚本**
7. 新建 `motor/can_ota.py`，实现 `_write_reg/_read_reg` + `download`。
8. 命令行对齐 `modbus_ota.py`，复用进度/结果打印。

**阶段 D — 联调与优化**
9. 硬件联调：先跑 md5 预检命中路径（直接 jump），再跑完整升级。
10. 优化（可选）：写 BLOCK 时不每帧等 ACK，连续发 32 帧后单次读 STATUS 确认，提升吞吐。
11. 优化（可选）：把 `can_id/master_id` 持久化到 `FIRMWARE_INFO` 页或独立 NV 字段。

---

## 9. 风险与缓解

| 风险 | 缓解 |
|---|---|
| MCAN `MCAN_Init` 默认 GFC=0x3F(拒收) | 必须显式 `MCAN_GlobalFilterConfig(ACCEPT_IN_RX_FIFO0,...)` + `MCAN_Start`（APP 版已验证） |
| 128B 块的 32 帧拼包错位 | SUB_OFFSET 严格 0..31，越界丢弃并 STATUS 置错；每帧 ACK 可让 host 及时发现 |
| bootloader 与 modbus 通路共用 `g_loader_ota` | 设计为同一时刻单通路使用；若并发，CONTROL 写入做状态校验（`ota->state` 不符则 STATUS 置 ENOTSUP） |
| host 等不到 ACK（CAN 总线错误/波特率不符） | `_write_reg` 带超时+重试，超时计入 `DownloadResult.*_FAILED` |
| 0x7FF 广播被总线上其他 DM 电机误收 | OTA 期间建议总线上只挂 host+待升级板；或后续用从机独有 ID（非广播）寻址（第二阶段） |

---

## 10. 交付物验收标准
- [ ] bootloader 仍能通过原 Modbus 通路升级（回归不破）。
- [ ] bootloader CAN 通路：`can_ota.py --firmware xxx.bin --id 1` 能完成 md5 预检→升级→跳转。
- [ ] md5 命中时直接跳转（不重写 flash）。
- [ ] 升级失败有明确 `DownloadResult` 枚举输出。
- [ ] `ota` / `damiao` 组件源码零改动。

---

**请确认本计划。** 确认后我按阶段 A→D 开始实施。如对 §2.1 的 RID 表、§2.4 的 CAN ID 方案、或 §8 的阶段划分有调整意见，请一并指出。
