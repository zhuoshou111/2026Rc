# STM32 麦轮小车与机械臂控制工程

本仓库包含 STM32F103ZE 麦轮底盘/机械臂固件、Keil 工程、上位机串口接口，以及可视化硬件调参工作台。当前正式固件入口是 `USER/chassis_controller_main.c`。

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

- 读取和精确写回 42 个硬件参数；
- Servo1/2/3、升降单项试动；
- 任务1、任务2动作试跑；
- mode 84（冠军台 36 mm）、85（亚军台 18 mm）、86（底部舵机回 24°）；
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
33/34/35             任务2舵盘到冠亚季军台
42/43/44             任务2地面奖杯到舵盘
80/81/82             Servo1/2/3 脉冲调试
83                   升降高度调试
84                   升降到冠军台 36 mm
85                   升降到亚军台 18 mm
86                   底部旋转舵机回 24°
```

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

## 安全说明

- 首次上电或复位后，固件中的软件升降高度从 `0 mm` 开始；执行升降/任务前应先把机构放到底部，并在工作台确认同步。
- 烧录前必须断开工作台串口，确认机械范围安全。
- 只有日志出现 `Verification OK` 才能判定烧录成功。
- 不要提交 `OBJ/`、BIN、Keil 中间文件、日志、Python 缓存或包含本机路径的调试配置。
