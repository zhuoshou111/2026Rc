# STM32 麦轮小车与机械臂控制工程

本仓库包含 STM32F103ZE 麦轮底盘/机械臂固件、Keil 工程、上位机串口接口，以及可视化硬件调参工作台。当前正式固件入口是 `USER/chassis_controller_main.c`。

当前源码基线为固件 `v149`。正式入口已经核对并清理：上电只进行安全 PWM、陀螺仪、升降 GPIO、底盘、串口和 PE6 输入初始化，不包含上电自动升降、自动转台、自动抓放、自动移动或一次性参数候选分支。mode 80/81/82/83 是调参工作台长期使用的受限试动接口，mode 126 是保留的陀螺仪诊断接口，均不属于上电自动测试；mode 89 启用比赛启动后会锁定三路舵机试动。

## 仓库结构

- `USER/Template.uvprojx`：Keil MDK 工程。
- `USER/chassis_controller_main.c`：UART1 指令监听、麦轮运动、升降、舵机、任务动作和陀螺仪逻辑。
- `HARDWARE/`、`SYSTEM/`、`CORE/`、`STM32F10x_FWLib/`：硬件驱动、系统支持、启动文件和 STM32 标准库。
- `stm32_hardware_tuning_gui.py`：参数读取/写回、串口试动、编译、导出 BIN 和烧录工作台。
- `upper_host_v86/`：供树莓派或其他上位机调用的 Python 串口接口。

`OBJ/`、`USER/Listings/`、`__pycache__/`、BIN、Keil 中间文件和日志均为生成物，不提交到 Git。

## 硬件与工具

- STM32F103ZE High-density，512 KiB Flash。
- Keil MDK-ARM 5.x，ARMCC 5.06 工具链，STM32F1xx DFP。
- Windows Python 3.10 或更高版本。
- Windows Tkinter（通常随 Python 安装）。
- 串口转换器/下载线。默认上位机串口为 `COM14`，波特率 `115200`。

## 安装 Python 依赖

在仓库根目录执行：

```powershell
python -m pip install -r requirements.txt
```

如果电脑安装了多个 Python，请使用实际启动工作台的解释器：

```powershell
py -3 -m pip install -r requirements.txt
```

依赖包括 `pyserial`、`loguru` 和 `stm32loader`。

## 启动调参工作台

双击 `启动硬件调参台.bat`，或执行：

```powershell
python stm32_hardware_tuning_gui.py
```

工作台默认读取 `USER/chassis_controller_main.c`，支持：

- 读取和精确写回 48 个硬件参数；
- Servo1/2/3、升降单项试动；
- 任务1、任务2动作试跑；
- 任务1/2放到舵盘、任务1在舵盘侧抓取和放回地面、任务2放到领奖台共用末段缓降参数，当前最后 `40 mm` 使用 `120 us` 半周期；
- mode 84（冠军台配置高度）、85（亚军台配置高度）、86（底部舵机回 24°）；
- 编译并检查 `0 Error(s), 0 Warning(s)`；
- 导出 BIN；
- 只对配置的串口进行烧录，并且只有出现 `Verification OK` 才报告成功。

工作台不会自动发送机械动作。执行试动或任务前，必须在界面中确认运动范围、升降零点和转台安全状态。

## 跨电脑配置

工作台不依赖某个用户的固定目录。Keil 工具按以下顺序寻找：

1. `KEIL_UV4`：直接指定 `UV4.exe`；
2. `KEIL_FROMELF`：直接指定 `fromelf.exe`；
3. `KEIL_ROOT`：指定 Keil 安装根目录；
4. 常见安装目录和系统 `PATH`。

换电脑或换 USB 端口时，可以设置：

```powershell
$env:KEIL_ROOT = 'C:\Keil_v5'
$env:STM32_SERIAL_PORT = 'COM7'
$env:STM32_SERIAL_BAUD = '115200'
python stm32_hardware_tuning_gui.py
```

默认仍使用 `COM14` 和 `115200`，当前设备无需额外配置。

## 编译固件

在 Keil 中打开 `USER/Template.uvprojx`，确认安装 `STM32F1xx_DFP` 后执行 Build。工作台的“仅编译并检查日志”按钮也会执行同一工程构建，并在需要时自动创建 `OBJ/` 和 `USER/Listings/`。

当前 `v149` 源码已于 `2026-08-29 17:13` 通过 Keil 编译，结果为 `0 Error(s), 0 Warning(s)`，并导出 `OBJ/chassis_hardware_tuning_v149_20260829_171302.bin`。该文件只表示源码构建产物；尚未执行烧录，不能据此声称板上已经运行 v149。

工程目标：

- Device：`STM32F103ZE`；
- Flash 起始地址：`0x08000000`；
- Flash 容量：`512 KiB`；
- 上位机 UART1：`115200 8N1`。

## 上位机接口

```python
from upper_host_v86 import MoveControl

control = MoveControl('COM14', 115200)
control.capToChampion()
control.capToRunnerUp()
control.turntable_home()
```

`MoveControl` 使用 `FF + 8 字节 payload + FE` 帧格式，并等待 STM32 返回 `FF 01 FE` 完成状态。换端口时，把构造函数的串口名改成目标端口。

## 重要模式

```text
1/2                 X/Y 厘米级移动
3                   旋转
5/6                 X/Y 毫米微调
7/8/9/10/12         地面抓取到舵盘
14/15/16/17/18      舵盘放回地面
33/34/35             任务2舵盘1/3/5到冠亚季军台
42/43/44             任务2地面奖杯到舵盘1/3/5
80/81/82             Servo1/2/3 脉冲调参试动
83                   升降高度调参试动
84                   升降到冠军台配置高度
85                   升降到亚军台配置高度
86                   底部旋转舵机回 24°
87                   启用任务2独立底盘速度和对称加减速
88                   恢复任务1底盘速度和运动方式
89                   初始化完成后启用 PE6 启动按键
```

mode 87/88 只切换当前上电周期内的底盘运动配置。mode 87 启用任务2独立长距巡航速度，并让任务2长距离、短距离、普通旋转和 mode 20 航向纠偏都使用对称的缓加速与缓减速曲线；短动作会自动把加减速脉冲数缩短到动作长度的一半。mode 88 完整恢复任务1原有长距、短距和旋转行为。mode 5/6 毫米微调始终使用独立固定速度，不受切换影响。

当前任务2初始配置为：长距巡航 `120 us`、短距巡航 `160 us`、旋转/纠偏巡航 `300 us`、起停端 `350 us`、加减速 `700` 脉冲。任务1参数保持长距/旋转 `70 us`、短距 `120 us`、长距起停 `220 us` 和 `700` 脉冲。mode 5/6 毫米微调固定为 `350 us`，不参与任务配置切换。

任务1和任务2共用的“从地面抓取并放到舵盘”收尾动作中，底部转台回正后等待 `500 ms` 即开始张开夹爪；其他 Servo3 动作仍使用 `850 ms` 等待。

mode 16（舵盘3搬运到地面）使用独立的“舵盘3取物张开位置”参数，当前值为 `1215 us`；mode 14/15/17/18 继续使用普通取舵盘位置。所有舵盘抓取后的正式夹紧仍统一使用 `SERVO2_CLAMP_PULSE_US`。

任务2的三个物块舵盘位置为舵盘1、3、5：mode 34/43 使用舵盘3。任务2从地面抓取后放到舵盘时，与任务1共用 `SERVO2_DISC_RELEASE_PULSE_US` 松开位置。任务1与任务2把物块放到舵盘时，以及任务1从舵盘取物时，舵盘侧下降的最后一段均使用与地面放置相同的缓降参数。

任务2把奖杯放到领奖台后，夹爪会先移动到 `SERVO2_BOTTOM_OPEN_PULSE_US` 对应的约45°全张开位置，等待舵机到位后升降机构才会上升；任务2在舵盘侧抓取奖杯前仍使用独立的 `SERVO2_TASK2_RELEASE_PULSE_US`。

## 当前正式机械参数

```text
Servo1 收回 / 任务2伸出：2500 / 946 us
Servo2 全张开 / 放舵盘 / 普通舵盘抓取前：765 / 1201 / 1202 us
Servo2 舵盘3抓取前 / 任务2抓取前 / 分段张开 / 夹紧：1215 / 1173 / 1018 / 1270 us
Servo3 HOME / 舵盘1 / 舵盘2 / 舵盘3 / 舵盘4 / 舵盘5：678 / 1675 / 1855 / 2014 / 2183 / 2352 us

地面抓取 / 地面放置 / 舵盘作业 / 转台安全高度：10 / 15 / 160 / 250 mm
任务结束待机 / 任务2回正安全高度：170 / 200 mm
冠军 / 亚军 / 季军台：40 / 22 / 4 mm
普通升降 / 末段缓降：15 / 120 us 半周期；末段距离 40 mm

Servo1 / Servo2 / Servo3 到位等待：200 / 380 / 850 ms
转台回正到张爪专用等待：500 ms
抓放保持 / 地面分段张开间隔：200 / 100 ms
```

## 上电启动流程

PE6 使用下拉输入，按下为高电平。上电后按键默认不触发任务；只有上位机完成机械初始化和陀螺仪归零并发送 mode 89 后，下一次有效按压才会上报 `FF 03 FE`。

```python
from upper_host_v86 import MoveControl

control = MoveControl("COM14", 115200)
control.initialize_and_wait_for_start()

# 收到 PE6 启动事件后，才从 Home 区域出发。
control.set_distance(x=0.6)
```

完整顺序为：等待 STM32 就绪 `FF 02 FE` → 机械初始化 mode 72 → 陀螺仪归零 mode 19 → 启用 PE6 mode 89 → 等待 `FF 03 FE` → 执行任务。mode 72 现在会在机械初始化真正完成后才返回 `FF 01 FE`，因此后续校准不会与初始化动作重叠。

## 引脚摘要

```text
上位机 UART1：PA9 TX、PA10 RX
陀螺仪 UART4：PC10 TX、PC11 RX
Servo1：TIM5_CH2 / PA1
Servo2：TIM5_CH3 / PA2
Servo3：TIM5_CH4 / PA3
升降 MOTOR1：PA4 DIR、PB10 STEP、PC5 EN
麦轮 STEP：PC6、PC7、PC8、PC9
```

Servo1/2/3 共用 TIM5，固件会先完成三通道配置并写入安全脉宽，再启动定时器，避免初始化阶段输出 0 us 无效脉冲。`HARDWARE/SERVO/servo.c` 保持 GBK/CP936 编码。Servo4 与 WS2812 都使用 TIM1_CH4 / PA11，存在资源冲突；当前正式入口不调用 `Servo4_Init()`。

## 安全说明

- 首次上电或复位后，固件中的软件升降高度从 `0 mm` 开始；执行升降/任务前应先把机构放到底部，并在工作台确认同步。
- 烧录前必须断开工作台串口，确认机械范围安全。
- 只有日志出现 `Verification OK` 才能判定烧录成功。
- 不要提交 `OBJ/`、BIN、Keil 中间文件、日志、Python 缓存或包含本机路径的调试配置。
