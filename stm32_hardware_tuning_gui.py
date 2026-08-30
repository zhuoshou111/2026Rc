#!/usr/bin/env python3
"""STM32F103ZE mecanum chassis and arm hardware tuning GUI.

The application never sends a command automatically.  Connecting, editing a
field, loading a profile, or saving source parameters cannot move hardware.
Mechanical movement happens only after the operator explicitly arms the UI,
clicks a test/task button, and accepts the confirmation dialog.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import queue
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable, Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - depends on the workstation
    serial = None
    list_ports = None


APP_TITLE = "STM32 麦轮小车与机械臂硬件调参台"
DEFAULT_ROOT = Path(__file__).resolve().parent
DEFAULT_SOURCE = DEFAULT_ROOT / "USER" / "chassis_controller_main.c"
PROJECT_PATH = DEFAULT_ROOT / "USER" / "Template.uvprojx"
OUTPUT_DIR = DEFAULT_ROOT / "OBJ"
LISTING_DIR = PROJECT_PATH.parent / "Listings"
BUILD_LOG_PATH = OUTPUT_DIR / "Template.build_log.htm"
AXF_PATH = OUTPUT_DIR / "Template.axf"


def _configured_path(name: str) -> Path | None:
    value = os.environ.get(name, "").strip()
    if not value:
        return None
    return Path(os.path.expandvars(os.path.expanduser(value)))


def _find_tool(env_name: str, executable: str, relative_to_keil_root: str) -> Path | None:
    configured = _configured_path(env_name)
    if configured is not None:
        return configured

    roots: list[Path] = []
    configured_root = _configured_path("KEIL_ROOT")
    if configured_root is not None:
        roots.append(configured_root)

    for variable in ("ProgramFiles", "ProgramFiles(x86)"):
        value = os.environ.get(variable, "").strip()
        if value:
            roots.append(Path(value) / "Keil_v5")

    roots.extend(
        Path(value)
        for value in (r"C:\Keil_v5", r"D:\Keil_v5", r"C:\Keil", r"D:\Keil")
    )
    for root in roots:
        candidate = root / relative_to_keil_root
        if candidate.exists():
            return candidate

    discovered = shutil.which(executable)
    return Path(discovered) if discovered else None


KEIL_PATH = _find_tool("KEIL_UV4", "UV4.exe", "UV4\\UV4.exe")
FROMELF_PATH = _find_tool("KEIL_FROMELF", "fromelf.exe", "ARM\\ARMCC\\bin\\fromelf.exe")
PYTHON_PATH = Path(sys.executable)
SERIAL_PORT = os.environ.get("STM32_SERIAL_PORT", "COM14").strip().upper() or "COM14"
try:
    SERIAL_BAUD = int(os.environ.get("STM32_SERIAL_BAUD", "115200"))
except ValueError:
    SERIAL_BAUD = 115200

FRAME_HEAD = 0xFF
FRAME_TAIL = 0xFE
PAYLOAD_SIZE = 8
STATUS_NAMES = {
    0x00: "命令无效",
    0x01: "动作完成",
    0x02: "控制器就绪",
    0x03: "PE6 启动按键已按下",
}

TUNING_MODE_SERVO1 = 80
TUNING_MODE_SERVO2 = 81
TUNING_MODE_SERVO3 = 82
TUNING_MODE_LIFT = 83
SERVO_TUNING_MIN_PULSE_US = 500
SERVO_TUNING_MAX_PULSE_US = 2500


@dataclass(frozen=True)
class ParameterSpec:
    define: str
    label: str
    group: str
    unit: str
    minimum: int
    maximum: int
    step: int
    actuator: str | None
    description: str


PARAMETERS = (
    ParameterSpec("SERVO1_RETRACT_PULSE_US", "收回位置", "Servo1 伸缩", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo1", "正式收回位置，当前标定约 270°"),
    ParameterSpec("SERVO1_TASK2_EXTEND_PULSE_US", "任务2前伸位置", "Servo1 伸缩", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo1", "奖杯放到讲台前的伸出位置，当前约 60°"),
    ParameterSpec("SERVO2_BOTTOM_OPEN_PULSE_US", "地面/领奖台全张开位置", "Servo2 夹爪", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "地面抓取、地面放置及任务2领奖台放置完成后使用，当前对应约45°"),
    ParameterSpec("SERVO2_DISC_RELEASE_PULSE_US", "任务1/2放舵盘位置", "Servo2 夹爪", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "任务1和任务2把物块放到舵盘时共用"),
    ParameterSpec("SERVO2_DISC_PICK_OPEN_PULSE_US", "任务1普通取舵盘位置", "Servo2 夹爪", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "任务1从舵盘1/2/4/5抓取前使用"),
    ParameterSpec("SERVO2_DISC3_PICK_OPEN_PULSE_US", "舵盘3取物张开位置", "Servo2 夹爪", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "仅 mode 16 舵盘3搬运到地面时，下降抓取前使用；正式夹紧仍使用统一夹紧位置"),
    ParameterSpec("SERVO2_TASK2_RELEASE_PULSE_US", "任务2取舵盘前张开位置", "Servo2 夹爪", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "仅在任务2下降到舵盘抓取奖杯前使用"),
    ParameterSpec("SERVO2_GROUND_RELEASE_STAGE_PULSE_US", "地面分段中间位置", "Servo2 夹爪", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "任务1放到地面时先松到该位置"),
    ParameterSpec("SERVO2_CLAMP_PULSE_US", "正式夹紧位置", "Servo2 夹爪", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo2", "所有任务的正式抓紧位置"),
    ParameterSpec("SERVO3_HOME_PULSE_US", "回正位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "机械臂正前方基准位置"),
    ParameterSpec("SERVO3_DISC1_PULSE_US", "舵盘1位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "任务1与任务2共用"),
    ParameterSpec("SERVO3_DISC2_PULSE_US", "舵盘2位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "任务1使用"),
    ParameterSpec("SERVO3_DISC3_PULSE_US", "舵盘3位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "任务1与任务2共用"),
    ParameterSpec("SERVO3_PUT_DISC4_PULSE_US", "舵盘4位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "任务1抓取与放置共用"),
    ParameterSpec("SERVO3_DISC5_PULSE_US", "舵盘5位置", "Servo3 底部旋转", "us", SERVO_TUNING_MIN_PULSE_US, SERVO_TUNING_MAX_PULSE_US, 5, "servo3", "任务1与任务2共用"),
    ParameterSpec("LIFT_GROUND_PICK_HEIGHT_MM", "地面抓取高度", "升降高度", "mm", 0, 270, 1, "lift", "从地面夹取物块或奖杯"),
    ParameterSpec("LIFT_GROUND_PLACE_HEIGHT_MM", "地面放置高度", "升降高度", "mm", 0, 270, 1, "lift", "任务1放回地面"),
    ParameterSpec("LIFT_GROUND_SLOW_APPROACH_DISTANCE_MM", "末段缓降距离", "升降高度", "mm", 1, 270, 5, None, "任务1/2放到舵盘、任务1从舵盘抓取和放回地面、任务2放上领奖台时，最后该距离减速"),
    ParameterSpec("LIFT_PICK_HEIGHT_MM", "待机结束高度", "升降高度", "mm", 0, 270, 1, "lift", "软件初始化和任务结束位置"),
    ParameterSpec("LIFT_PLACE_HEIGHT_MM", "任务1舵盘高度", "升降高度", "mm", 0, 270, 1, "lift", "任务1舵盘抓取和放置"),
    ParameterSpec("LIFT_TRANSFER_HEIGHT_MM", "转台旋转安全高度", "升降高度", "mm", 0, 270, 1, "lift", "任务1转台旋转前的高度"),
    ParameterSpec("LIFT_MID_HEIGHT_MM", "中位试动高度", "升降高度", "mm", 0, 270, 1, "lift", "独立升降调参位置"),
    ParameterSpec("LIFT_178_HEIGHT_MM", "178mm 试动高度", "升降高度", "mm", 0, 270, 1, "lift", "独立升降调参位置"),
    ParameterSpec("LIFT_186_HEIGHT_MM", "186mm 试动高度", "升降高度", "mm", 0, 270, 1, "lift", "独立升降调参位置"),
    ParameterSpec("LIFT_TASK2_DISC_HEIGHT_MM", "任务2舵盘高度", "升降高度", "mm", 0, 270, 1, "lift", "奖杯在舵盘上的抓取/放置高度"),
    ParameterSpec("LIFT_TASK2_RETURN_CLEARANCE_MM", "任务2回正安全高度", "升降高度", "mm", 0, 270, 1, "lift", "任务2取杯后转台回正前高度"),
    ParameterSpec("LIFT_TASK2_CHAMPION_HEIGHT_MM", "冠军讲台高度", "升降高度", "mm", 0, 270, 1, "lift", "任务2最终放置高度"),
    ParameterSpec("LIFT_TASK2_RUNNER_UP_HEIGHT_MM", "亚军讲台高度", "升降高度", "mm", 0, 270, 1, "lift", "任务2最终放置高度"),
    ParameterSpec("LIFT_TASK2_THIRD_PLACE_HEIGHT_MM", "季军讲台高度", "升降高度", "mm", 0, 270, 1, "lift", "任务2最终放置高度"),
    ParameterSpec("CHASSIS_STEP_HALF_PERIOD_US", "任务1长距/旋转速度", "速度与延时", "us", 40, 500, 5, None, "mode 88 下任务1长距离巡航和普通旋转共用；数值越小越快"),
    ParameterSpec("CHASSIS_SHORT_STEP_HALF_PERIOD_US", "任务1短距直线速度", "速度与延时", "us", 40, 1000, 10, None, "mode 88 下小于长距阈值的厘米级直线移动；数值越小越快"),
    ParameterSpec("CHASSIS_RAMP_START_HALF_PERIOD_US", "任务1长距起步停车速度", "速度与延时", "us", 50, 1000, 10, None, "任务1长距离的起步和停车速度；数值越大越慢"),
    ParameterSpec("CHASSIS_RAMP_PULSES", "任务1长距加减速脉冲数", "速度与延时", "pulse", 2, 10000, 50, None, "任务1长距离起步和刹停各使用该脉冲数"),
    ParameterSpec("CHASSIS_TASK2_LONG_STEP_HALF_PERIOD_US", "任务2长距巡航速度", "速度与延时", "us", 40, 1000, 10, None, "mode 87 下长距离移动的独立巡航速度；数值越小越快"),
    ParameterSpec("CHASSIS_SLOW_SHORT_STEP_HALF_PERIOD_US", "任务2短距巡航速度", "速度与延时", "us", 40, 1000, 10, None, "mode 87 下厘米级短距离直线移动的巡航速度；起步和停车都使用加减速曲线"),
    ParameterSpec("CHASSIS_ROTATE_SLOW_STEP_HALF_PERIOD_US", "任务2旋转/纠偏速度", "速度与延时", "us", 40, 1000, 10, None, "mode 87 下普通旋转和 mode 20 航向纠偏共用；起步和停车都使用加减速曲线"),
    ParameterSpec("CHASSIS_TASK2_STOP_HALF_PERIOD_US", "任务2起停端速度", "速度与延时", "us", 50, 1500, 10, None, "任务2长距、短距、旋转和纠偏均从该半周期缓加速，并缓减速回该半周期"),
    ParameterSpec("CHASSIS_TASK2_RAMP_PULSES", "任务2加减速脉冲数", "速度与延时", "pulse", 2, 10000, 50, None, "任务2长距、短距、旋转和纠偏的起步与停车两端共用；短动作会自动缩短"),
    ParameterSpec("CHASSIS_FINE_STEP_HALF_PERIOD_US", "毫米微调速度", "速度与延时", "us", 40, 1000, 10, None, "毫米级微调专用固定速度；数值越小越快"),
    ParameterSpec("CHASSIS_RAMP_MIN_DISTANCE_MM", "长距离判定阈值", "速度与延时", "mm", 1, 2000, 10, None, "任务1和任务2达到该距离时使用各自的长距加减速配置"),
    ParameterSpec("LIFT_STEP_HALF_PERIOD_US", "普通升降速度", "速度与延时", "us", 15, 500, 5, None, "常规升降速度；数值越小，升降越快"),
    ParameterSpec("LIFT_GROUND_SLOW_STEP_HALF_PERIOD_US", "末段缓降速度", "速度与延时", "us", 15, 1000, 10, None, "任务1/2放到舵盘、任务1从舵盘抓取和放回地面、任务2放上领奖台的最后一段共用；数值越大越慢"),
    ParameterSpec("SERVO1_SETTLE_MS", "伸缩动作等待", "速度与延时", "ms", 0, 5000, 50, None, "每次 Servo1 调整后的等待时间"),
    ParameterSpec("SERVO2_SETTLE_MS", "夹爪动作等待", "速度与延时", "ms", 0, 5000, 50, None, "每次夹爪调整后的等待时间"),
    ParameterSpec("SERVO3_SETTLE_MS", "转台动作等待", "速度与延时", "ms", 0, 5000, 50, None, "每次转台调整后的等待时间"),
    ParameterSpec("SERVO3_HOME_BEFORE_CLAW_OPEN_MS", "回正后张爪等待", "速度与延时", "ms", 0, 5000, 50, None, "任务1/2从地面抓取并放到舵盘后，转台回正到夹爪张开的专用等待；不影响其他转台动作"),
    ParameterSpec("ARM_ACTION_HOLD_MS", "抓放保持时间", "速度与延时", "ms", 0, 5000, 50, None, "夹紧或松开后的额外保持"),
    ParameterSpec("GROUND_RELEASE_STAGE_INTERVAL_MS", "地面分段张开间隔", "速度与延时", "ms", 0, 1000, 10, None, "中间松开到完全张开之间的时间"),
)

PARAMETER_BY_DEFINE = {item.define: item for item in PARAMETERS}
GROUPS = ("Servo1 伸缩", "Servo2 夹爪", "Servo3 底部旋转", "升降高度", "速度与延时")


TASK_GROUPS = (
    (
        "任务1：地面 → 舵盘",
        (("舵盘1", 7), ("舵盘2", 8), ("舵盘3", 9), ("舵盘4", 10), ("舵盘5", 12)),
    ),
    (
        "任务1：舵盘 → 地面",
        (("舵盘1", 14), ("舵盘2", 15), ("舵盘3", 16), ("舵盘4", 17), ("舵盘5", 18)),
    ),
    (
        "任务2：地面奖杯 → 舵盘",
        (("奖杯1→舵盘1", 42), ("奖杯2→舵盘3", 43), ("奖杯3→舵盘5", 44)),
    ),
    (
        "任务2：舵盘 → 讲台",
        (("舵盘1→冠军", 33), ("舵盘3→亚军", 34), ("舵盘5→季军", 35)),
    ),
)

MANUAL_ACTIONS = (
    ("软件初始化", 72, True),
    ("Servo1 伸到任务2位置", 36, False),
    ("Servo1 收回", 52, False),
    ("夹爪张开", 70, False),
    ("升到 270 mm", 48, True),
    ("降到 0 mm", 49, True),
    ("升到 mode 53 高度", 53, True),
    ("升到 178 mm", 58, True),
    ("升到 186 mm", 59, True),
    ("升到冠军台配置高度", 84, True),
    ("升到亚军台配置高度", 85, True),
    ("底部旋转舵机回 24°", 86, False),
    ("启用任务2底盘配置", 87, False),
    ("恢复任务1底盘配置", 88, False),
    ("启用 PE6 启动按键", 89, False),
    ("记录陀螺仪基准", 19, False),
    ("底盘航向纠偏", 20, False),
)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def make_frame(mode: int, value: int | None = None) -> bytes:
    if not 0 <= mode <= 255:
        raise ValueError("mode 必须在 0–255 范围内")
    payload = [0] * PAYLOAD_SIZE
    payload[0] = mode
    if value is not None:
        if not 0 <= value <= 65535:
            raise ValueError("参数必须在 0–65535 范围内")
        payload[1] = (value >> 8) & 0xFF
        payload[2] = value & 0xFF
    return bytes([FRAME_HEAD, *payload, FRAME_TAIL])


def hex_bytes(data: Iterable[int]) -> str:
    return " ".join(f"{value:02X}" for value in data)


def parse_source_parameters(source_path: Path) -> tuple[dict[str, int], str]:
    raw = source_path.read_bytes()
    values: dict[str, int] = {}
    for spec in PARAMETERS:
        pattern = re.compile(
            rb"(?m)^\s*#define\s+" + re.escape(spec.define.encode("ascii")) + rb"\s+(\d+)u\b"
        )
        match = pattern.search(raw)
        if match is None:
            raise ValueError(f"源码中找不到参数：{spec.define}")
        values[spec.define] = int(match.group(1))
    return values, sha256_bytes(raw)


def read_firmware_version(source_path: Path) -> int:
    raw = source_path.read_bytes()
    match = re.search(
        rb"(?m)^\s*#define\s+FIRMWARE_DIAGNOSTIC_VERSION\s+(\d+)u\b", raw
    )
    if match is None:
        raise ValueError("源码中找不到 FIRMWARE_DIAGNOSTIC_VERSION")
    return int(match.group(1))


def replace_source_parameters(
    source_path: Path, expected_hash: str, values: dict[str, int]
) -> str:
    raw = source_path.read_bytes()
    if sha256_bytes(raw) != expected_hash:
        raise RuntimeError("源码已被其他程序修改。为避免覆盖现有改动，请先重新读取源码。")

    updated = raw
    for spec in PARAMETERS:
        value = values[spec.define]
        pattern = re.compile(
            rb"(?m)^(\s*#define\s+"
            + re.escape(spec.define.encode("ascii"))
            + rb"\s+)(\d+)(u\b)"
        )
        replacement = lambda match, new=str(value).encode("ascii"): (
            match.group(1) + new + match.group(3)
        )
        updated, count = pattern.subn(replacement, updated, count=1)
        if count != 1:
            raise RuntimeError(f"参数替换失败：{spec.define}")

    if updated == raw:
        return expected_hash

    temp_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", prefix=source_path.name + ".", suffix=".tmp", dir=source_path.parent, delete=False
        ) as handle:
            handle.write(updated)
            handle.flush()
            os.fsync(handle.fileno())
            temp_name = handle.name
        os.replace(temp_name, source_path)
        temp_name = None
    finally:
        if temp_name is not None:
            try:
                os.unlink(temp_name)
            except OSError:
                pass
    return sha256_bytes(updated)


def approximate_degrees(actuator: str, pulse_us: int) -> float:
    zero = 426.0 if actuator == "servo2" else 500.0
    return (pulse_us - zero) * 270.0 / 2000.0


class SerialWorker:
    """Own the configured serial port in a worker thread."""

    def __init__(self, events: queue.Queue[dict[str, object]]) -> None:
        self.events = events
        self.requests: queue.Queue[tuple[str, object]] = queue.Queue()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def request(self, action: str, data: object = None) -> None:
        self.requests.put((action, data))

    def _emit(self, event: str, **data: object) -> None:
        self.events.put({"event": event, **data})

    def _run(self) -> None:
        port = None
        rx_buffer = bytearray()
        pending: dict[str, object] | None = None
        running = True

        while running:
            try:
                while True:
                    action, data = self.requests.get_nowait()
                    if action == "connect":
                        if port is not None:
                            port.close()
                        try:
                            port = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
                            port.reset_input_buffer()
                            port.reset_output_buffer()
                            rx_buffer.clear()
                            pending = None
                            self._emit("connected")
                        except Exception as exc:
                            port = None
                            self._emit("serial_error", message=f"打开 {SERIAL_PORT} 失败：{exc}")
                    elif action == "disconnect":
                        if port is not None:
                            port.close()
                        port = None
                        pending = None
                        rx_buffer.clear()
                        self._emit("disconnected")
                    elif action == "send":
                        frame, label, timeout_s = data  # type: ignore[misc]
                        if port is None:
                            self._emit("serial_error", message=f"请先连接 {SERIAL_PORT}。")
                            self._emit("operation_done")
                            continue
                        if pending is not None:
                            self._emit("serial_error", message="上一条命令仍在等待返回。")
                            self._emit("operation_done")
                            continue
                        try:
                            port.write(frame)
                            port.flush()
                            now = time.monotonic()
                            pending = {
                                "label": label,
                                "started": now,
                                "deadline": now + float(timeout_s),
                            }
                            self._emit("tx", frame=frame, label=label)
                        except Exception as exc:
                            self._emit("serial_error", message=f"发送失败：{exc}")
                            self._emit("operation_done")
                    elif action == "stop":
                        running = False
                        break
            except queue.Empty:
                pass
            except Exception as exc:
                self._emit("serial_error", message=f"串口线程异常：{exc}")

            if not running:
                break

            if port is not None:
                try:
                    waiting = port.in_waiting
                    if waiting:
                        rx_buffer.extend(port.read(waiting))
                    pending = self._parse_status(rx_buffer, pending)
                except Exception as exc:
                    try:
                        port.close()
                    except Exception:
                        pass
                    port = None
                    pending = None
                    self._emit("serial_error", message=f"{SERIAL_PORT} 连接中断：{exc}")
                    self._emit("disconnected")

            if pending is not None and time.monotonic() >= float(pending["deadline"]):
                elapsed = time.monotonic() - float(pending["started"])
                self._emit("timeout", label=pending["label"], elapsed=elapsed)
                pending = None
                self._emit("operation_done")

            time.sleep(0.01)

        if port is not None:
            port.close()

    def _parse_status(
        self, rx_buffer: bytearray, pending: dict[str, object] | None
    ) -> dict[str, object] | None:
        while len(rx_buffer) >= 3:
            try:
                head = rx_buffer.index(FRAME_HEAD)
            except ValueError:
                self._emit("noise", data=bytes(rx_buffer))
                rx_buffer.clear()
                return pending
            if head:
                self._emit("noise", data=bytes(rx_buffer[:head]))
                del rx_buffer[:head]
            if len(rx_buffer) < 3:
                break
            if rx_buffer[2] != FRAME_TAIL:
                self._emit("noise", data=bytes([rx_buffer[0]]))
                del rx_buffer[0]
                continue
            frame = bytes(rx_buffer[:3])
            del rx_buffer[:3]
            status = frame[1]
            elapsed = None
            if pending is not None:
                elapsed = time.monotonic() - float(pending["started"])
            self._emit("rx", frame=frame, status=status, elapsed=elapsed)
            if pending is not None and status in (0x00, 0x01):
                pending = None
                self._emit("operation_done")
        return pending


class HardwareTuningApp(tk.Tk):
    def __init__(self, source_path: Path) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1240x820")
        self.minsize(1000, 680)

        self.source_path = source_path.resolve()
        self.events: queue.Queue[dict[str, object]] = queue.Queue()
        self.worker = SerialWorker(self.events) if serial is not None else None
        self.connected = False
        self.busy = False
        self.build_busy = False
        self.loaded_hash = ""
        self.loaded_values: dict[str, int] = {}
        self.value_vars = {spec.define: tk.StringVar() for spec in PARAMETERS}
        self.degree_vars: dict[str, tk.StringVar] = {}
        self.servo2_manual_var = tk.StringVar(value="")
        self.servo2_manual_degree_var = tk.StringVar(value="")
        self.servo2_manual_var.trace_add("write", lambda *_args: self._update_servo2_manual_degree())
        self.action_armed_var = tk.BooleanVar(value=False)
        self.lift_synced_var = tk.BooleanVar(value=False)
        self.turntable_safe_var = tk.BooleanVar(value=False)
        self.connection_var = tk.StringVar(value="未连接")
        self.port_scan_var = tk.StringVar(value="尚未扫描")
        self.status_var = tk.StringVar(value="等待操作")
        self.dirty_var = tk.StringVar(value="")
        self.bin_name_var = tk.StringVar(value="")
        self.action_buttons: list[ttk.Button] = []

        self._configure_style()
        self._build_ui()
        self._load_source(show_message=False)
        self._refresh_ports()
        self.after(50, self._poll_events)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        if serial is None:
            self.after(
                100,
                lambda: messagebox.showerror(
                    "缺少组件", "未安装 pyserial。请在仓库根目录运行：\npython -m pip install -r requirements.txt"
                ),
            )

    def _configure_style(self) -> None:
        style = ttk.Style(self)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 17, "bold"))
        style.configure("Status.TLabel", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Safe.TLabel", foreground="#147d3f")
        style.configure("Warn.TLabel", foreground="#a04b00")
        style.configure("Accent.TButton", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Danger.TButton", foreground="#a32020")

    def _build_ui(self) -> None:
        outer = ttk.Frame(self, padding=12)
        outer.pack(fill="both", expand=True)

        title_row = ttk.Frame(outer)
        title_row.pack(fill="x", pady=(0, 8))
        ttk.Label(title_row, text=APP_TITLE, style="Title.TLabel").pack(side="left")
        ttk.Label(title_row, textvariable=self.status_var, style="Status.TLabel").pack(side="right")

        connection = ttk.LabelFrame(outer, text=f"{SERIAL_PORT} 连接与机械安全", padding=9)
        connection.pack(fill="x", pady=(0, 8))
        ttk.Label(connection, text=f"当前串口：{SERIAL_PORT} @ {SERIAL_BAUD}").grid(row=0, column=0, sticky="w")
        ttk.Button(connection, text="扫描", command=self._refresh_ports, width=8).grid(row=0, column=1, padx=5)
        self.connect_button = ttk.Button(connection, text="连接", command=self._toggle_connection, width=10)
        self.connect_button.grid(row=0, column=2, padx=5)
        ttk.Label(connection, textvariable=self.connection_var, style="Status.TLabel").grid(row=0, column=3, padx=(8, 18), sticky="w")
        ttk.Label(connection, textvariable=self.port_scan_var).grid(row=0, column=4, sticky="w")
        connection.columnconfigure(4, weight=1)

        ttk.Checkbutton(
            connection,
            text="我已清空运动范围，允许本次会话发送机械动作",
            variable=self.action_armed_var,
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Checkbutton(
            connection,
            text="升降已人工放到底，软件 0 mm 与实际位置已同步",
            variable=self.lift_synced_var,
        ).grid(row=1, column=2, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Checkbutton(
            connection,
            text="转台旋转区域无干涉/机械臂已在安全高度",
            variable=self.turntable_safe_var,
        ).grid(row=1, column=4, sticky="w", pady=(8, 0))

        self.notebook = ttk.Notebook(outer)
        self.notebook.pack(fill="both", expand=True)
        parameter_tab = ttk.Frame(self.notebook, padding=8)
        task_tab = ttk.Frame(self.notebook, padding=8)
        build_tab = ttk.Frame(self.notebook, padding=8)
        log_tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(parameter_tab, text="参数标定")
        self.notebook.add(task_tab, text="任务试跑")
        self.notebook.add(build_tab, text="保存 / 编译 / 烧录")
        self.notebook.add(log_tab, text="通信与构建日志")

        self._build_parameter_tab(parameter_tab)
        self._build_task_tab(task_tab)
        self._build_build_tab(build_tab)
        self._build_log_tab(log_tab)

    def _build_parameter_tab(self, parent: ttk.Frame) -> None:
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill="x", pady=(0, 8))
        ttk.Button(toolbar, text="重新读取源码", command=self._load_source).pack(side="left")
        ttk.Button(toolbar, text="预览修改", command=self._preview_changes).pack(side="left", padx=6)
        ttk.Button(toolbar, text="导出参数方案", command=self._export_profile).pack(side="left", padx=6)
        ttk.Button(toolbar, text="导入参数方案", command=self._import_profile).pack(side="left", padx=6)
        ttk.Label(toolbar, textvariable=self.dirty_var, style="Warn.TLabel").pack(side="right")

        manual = ttk.LabelFrame(parent, text="Servo2 夹爪自由试动（不修改正式任务参数）", padding=8)
        manual.pack(fill="x", pady=(0, 8))
        ttk.Label(manual, text="手动脉宽").pack(side="left")
        ttk.Spinbox(
            manual,
            textvariable=self.servo2_manual_var,
            from_=SERVO_TUNING_MIN_PULSE_US,
            to=SERVO_TUNING_MAX_PULSE_US,
            increment=1,
            width=10,
        ).pack(side="left", padx=(8, 4))
        ttk.Label(manual, textvariable=self.servo2_manual_degree_var, width=18).pack(side="left")
        for delta in (-10, -1, 1, 10):
            sign = "+" if delta > 0 else ""
            ttk.Button(
                manual,
                text=f"{sign}{delta}",
                width=5,
                command=lambda amount=delta: self._nudge_servo2_manual(amount),
            ).pack(side="left", padx=(3, 0))
        button = ttk.Button(manual, text="试动夹爪", command=self._test_servo2_manual, width=12)
        button.pack(side="left", padx=(10, 8))
        self.action_buttons.append(button)
        ttk.Label(
            manual,
            text=(
                f"固件允许范围 {SERVO_TUNING_MIN_PULSE_US}–{SERVO_TUNING_MAX_PULSE_US} us；"
                "输入初值自动读取当前正式夹紧参数，请小步试动"
            ),
            style="Safe.TLabel",
        ).pack(side="left")

        host = ttk.Frame(parent)
        host.pack(fill="both", expand=True)
        canvas = tk.Canvas(host, highlightthickness=0)
        scroll = ttk.Scrollbar(host, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set)
        canvas.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        body = ttk.Frame(canvas, padding=(0, 0, 8, 0))
        window = canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(window, width=e.width))
        canvas.bind("<MouseWheel>", lambda e: canvas.yview_scroll(int(-e.delta / 120), "units"))

        for group in GROUPS:
            frame = ttk.LabelFrame(body, text=group, padding=8)
            frame.pack(fill="x", pady=(0, 8))
            headings = ("参数", "数值", "单位/角度", "微调", "单项试动", "用途")
            for column, heading in enumerate(headings):
                ttk.Label(frame, text=heading, style="Status.TLabel").grid(
                    row=0, column=column, sticky="w", padx=4, pady=(0, 4)
                )
            frame.columnconfigure(5, weight=1)
            row = 1
            for spec in PARAMETERS:
                if spec.group != group:
                    continue
                self._parameter_row(frame, row, spec)
                row += 1

    def _parameter_row(self, parent: ttk.LabelFrame, row: int, spec: ParameterSpec) -> None:
        variable = self.value_vars[spec.define]
        variable.trace_add("write", lambda *_args: self._on_parameter_changed())
        ttk.Label(parent, text=spec.label, width=24).grid(row=row, column=0, sticky="w", padx=4, pady=3)
        ttk.Spinbox(
            parent,
            textvariable=variable,
            from_=spec.minimum,
            to=spec.maximum,
            increment=spec.step,
            width=10,
        ).grid(row=row, column=1, sticky="w", padx=4, pady=3)

        if spec.actuator and spec.actuator.startswith("servo"):
            degree_var = tk.StringVar(value="")
            self.degree_vars[spec.define] = degree_var
            ttk.Label(parent, textvariable=degree_var, width=15).grid(row=row, column=2, sticky="w", padx=4)
        else:
            ttk.Label(parent, text=spec.unit, width=15).grid(row=row, column=2, sticky="w", padx=4)

        adjust = ttk.Frame(parent)
        adjust.grid(row=row, column=3, sticky="w", padx=4)
        ttk.Button(adjust, text=f"−{spec.step}", width=6, command=lambda s=spec: self._nudge(s, -s.step)).pack(side="left")
        ttk.Button(adjust, text=f"+{spec.step}", width=6, command=lambda s=spec: self._nudge(s, s.step)).pack(side="left", padx=(3, 0))

        if spec.actuator is not None:
            button = ttk.Button(parent, text="试动此值", command=lambda s=spec: self._test_parameter(s), width=11)
            button.grid(row=row, column=4, sticky="w", padx=4)
            self.action_buttons.append(button)
        else:
            ttk.Label(parent, text="保存后生效", foreground="#666666").grid(row=row, column=4, sticky="w", padx=4)
        ttk.Label(parent, text=spec.description).grid(row=row, column=5, sticky="w", padx=4)

    def _build_task_tab(self, parent: ttk.Frame) -> None:
        warning = ttk.LabelFrame(parent, text="操作前确认", padding=10)
        warning.pack(fill="x", pady=(0, 10))
        ttk.Label(
            warning,
            text="任务动作是阻塞执行；收到完成 ACK 前界面不会允许发送下一条。执行任务前必须先人工将升降机构放到最低点并确认软件高度同步。",
            style="Warn.TLabel",
            wraplength=1050,
        ).pack(anchor="w")

        for title, actions in TASK_GROUPS:
            frame = ttk.LabelFrame(parent, text=title, padding=10)
            frame.pack(fill="x", pady=6)
            for index, (label, mode) in enumerate(actions):
                button = ttk.Button(
                    frame,
                    text=f"{label}\nmode {mode}",
                    command=lambda m=mode, text=label: self._send_mode_action(m, text, needs_lift=True),
                    width=18,
                )
                button.grid(row=0, column=index, sticky="ew", padx=5, pady=3)
                frame.columnconfigure(index, weight=1)
                self.action_buttons.append(button)

        manual = ttk.LabelFrame(parent, text="独立动作与初始化", padding=10)
        manual.pack(fill="x", pady=6)
        for index, (label, mode, needs_lift) in enumerate(MANUAL_ACTIONS):
            button = ttk.Button(
                manual,
                text=f"{label}\nmode {mode}",
                command=lambda m=mode, text=label, lift=needs_lift: self._send_mode_action(m, text, needs_lift=lift),
                width=18,
            )
            button.grid(row=index // 4, column=index % 4, sticky="ew", padx=5, pady=5)
            manual.columnconfigure(index % 4, weight=1)
            self.action_buttons.append(button)

    def _build_build_tab(self, parent: ttk.Frame) -> None:
        source_frame = ttk.LabelFrame(parent, text="参数写回", padding=12)
        source_frame.pack(fill="x", pady=(0, 10))
        ttk.Label(source_frame, text=f"目标源码：{self.source_path}", wraplength=1050).pack(anchor="w")
        ttk.Label(
            source_frame,
            text="写回时只替换所列 #define 的十进制数字，并检查文件哈希；不会整体解码或转码，也不会覆盖加载后产生的外部修改。",
            style="Safe.TLabel",
            wraplength=1050,
        ).pack(anchor="w", pady=(5, 10))
        ttk.Button(source_frame, text="预览修改", command=self._preview_changes).pack(side="left")
        ttk.Button(source_frame, text="保存参数到当前 .c", command=self._save_source, style="Accent.TButton").pack(side="left", padx=8)

        build_frame = ttk.LabelFrame(parent, text="构建与 BIN", padding=12)
        build_frame.pack(fill="x", pady=10)
        ttk.Label(build_frame, text=f"Keil 工程：{PROJECT_PATH}").grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(build_frame, text="新 BIN 名称").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(build_frame, textvariable=self.bin_name_var, width=62).grid(row=1, column=1, sticky="ew", padx=8, pady=(8, 0))
        ttk.Label(build_frame, text="留空时自动生成").grid(row=1, column=2, sticky="w", pady=(8, 0))
        build_frame.columnconfigure(1, weight=1)
        ttk.Button(build_frame, text="仅编译并检查日志", command=lambda: self._start_build(False, False)).grid(row=2, column=0, padx=4, pady=12, sticky="ew")
        ttk.Button(build_frame, text="编译并导出 BIN", command=lambda: self._start_build(True, False)).grid(row=2, column=1, padx=4, pady=12, sticky="ew")
        ttk.Button(build_frame, text=f"编译、导出并烧录 {SERIAL_PORT}", command=lambda: self._start_build(True, True), style="Danger.TButton").grid(row=2, column=2, padx=4, pady=12, sticky="ew")

        flash_frame = ttk.LabelFrame(parent, text="烧录硬性规则", padding=12)
        flash_frame.pack(fill="x", pady=10)
        ttk.Label(
            flash_frame,
            text=f"只允许 {SERIAL_PORT}；{SERIAL_PORT} 不存在立即停止。烧录前必须断开界面串口。只有日志明确出现 Verification OK，界面才会报告烧录成功。烧录后单片机会复位，请确认机械范围安全。",
            style="Warn.TLabel",
            wraplength=1050,
        ).pack(anchor="w")

    def _build_log_tab(self, parent: ttk.Frame) -> None:
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill="x", pady=(0, 6))
        ttk.Label(toolbar, text="不会自动重发超时命令，避免机械动作重复执行。", style="Safe.TLabel").pack(side="left")
        ttk.Button(toolbar, text="清空日志", command=self._clear_log).pack(side="right")
        self.log = tk.Text(
            parent,
            state="disabled",
            wrap="word",
            font=("Cascadia Mono", 9),
            background="#101820",
            foreground="#e8eef2",
            padx=8,
            pady=8,
        )
        scroll = ttk.Scrollbar(parent, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.log.pack(fill="both", expand=True)
        self.log.tag_configure("tx", foreground="#69c0ff")
        self.log.tag_configure("rx", foreground="#7ee787")
        self.log.tag_configure("warn", foreground="#ffcc66")
        self.log.tag_configure("error", foreground="#ff7b72")
        self.log.tag_configure("info", foreground="#b9c4cc")
        self._append_log("调参台已启动；尚未发送任何机械动作。", "info")

    def _refresh_ports(self) -> None:
        ports = [] if list_ports is None else [item.device.upper() for item in list_ports.comports()]
        if SERIAL_PORT in ports:
            self.port_scan_var.set(f"已检测到 {SERIAL_PORT}")
        else:
            self.port_scan_var.set(f"未检测到 {SERIAL_PORT}，请重新插拔 USB")
        self._append_log("串口扫描：" + (", ".join(ports) if ports else "未发现串口"), "info")

    def _toggle_connection(self) -> None:
        if self.worker is None:
            messagebox.showerror("缺少组件", "未安装 pyserial。")
            return
        if self.connected:
            self.worker.request("disconnect")
            return
        ports = [] if list_ports is None else [item.device.upper() for item in list_ports.comports()]
        if SERIAL_PORT not in ports:
            messagebox.showwarning(f"{SERIAL_PORT} 不存在", "请重新插拔 USB；不会改用其他串口。")
            return
        self.connection_var.set("正在连接…")
        self.connect_button.configure(state="disabled")
        self.worker.request("connect")

    def _load_source(self, show_message: bool = True) -> None:
        try:
            values, source_hash = parse_source_parameters(self.source_path)
        except Exception as exc:
            messagebox.showerror("读取源码失败", str(exc))
            return
        self.loaded_values = values
        self.loaded_hash = source_hash
        for define, value in values.items():
            self.value_vars[define].set(str(value))
        self.servo2_manual_var.set(str(values["SERVO2_CLAMP_PULSE_US"]))
        self._update_degree_labels()
        self._update_dirty_state()
        try:
            version = read_firmware_version(self.source_path)
            self.bin_name_var.set(f"chassis_hardware_tuning_v{version}.bin")
        except Exception:
            pass
        self._append_log(f"已读取源码参数，SHA-256 {source_hash[:12]}…", "info")
        if show_message:
            messagebox.showinfo("读取完成", "已从当前 .c 重新载入全部调参参数。")

    def _validated_values(self) -> dict[str, int] | None:
        values: dict[str, int] = {}
        errors: list[str] = []
        for spec in PARAMETERS:
            text = self.value_vars[spec.define].get().strip()
            try:
                value = int(text, 10)
            except ValueError:
                errors.append(f"{spec.label}：必须是整数")
                continue
            if not spec.minimum <= value <= spec.maximum:
                errors.append(f"{spec.label}：允许范围 {spec.minimum}–{spec.maximum} {spec.unit}")
                continue
            values[spec.define] = value
        if (
            "CHASSIS_STEP_HALF_PERIOD_US" in values
            and "CHASSIS_RAMP_START_HALF_PERIOD_US" in values
            and values["CHASSIS_RAMP_START_HALF_PERIOD_US"]
            <= values["CHASSIS_STEP_HALF_PERIOD_US"]
        ):
            errors.append("任务1长距起步停车半周期必须大于任务1巡航半周期")
        task2_stop = values.get("CHASSIS_TASK2_STOP_HALF_PERIOD_US")
        for define, label in (
            ("CHASSIS_TASK2_LONG_STEP_HALF_PERIOD_US", "任务2长距巡航"),
            ("CHASSIS_SLOW_SHORT_STEP_HALF_PERIOD_US", "任务2短距巡航"),
            ("CHASSIS_ROTATE_SLOW_STEP_HALF_PERIOD_US", "任务2旋转/纠偏"),
        ):
            if (
                task2_stop is not None
                and define in values
                and task2_stop <= values[define]
            ):
                errors.append(f"任务2起停端半周期必须大于{label}半周期")
        if (
            "LIFT_STEP_HALF_PERIOD_US" in values
            and "LIFT_GROUND_SLOW_STEP_HALF_PERIOD_US" in values
            and values["LIFT_GROUND_SLOW_STEP_HALF_PERIOD_US"]
            <= values["LIFT_STEP_HALF_PERIOD_US"]
        ):
            errors.append("放置末段慢降半周期必须大于普通升降半周期")
        if errors:
            messagebox.showerror("参数错误", "\n".join(errors[:12]))
            return None
        return values

    def _on_parameter_changed(self) -> None:
        self._update_degree_labels()
        self._update_dirty_state()

    def _update_degree_labels(self) -> None:
        for spec in PARAMETERS:
            if spec.define not in self.degree_vars or spec.actuator is None:
                continue
            try:
                pulse = int(self.value_vars[spec.define].get().strip())
                degrees = approximate_degrees(spec.actuator, pulse)
                self.degree_vars[spec.define].set(f"{spec.unit}  ≈ {degrees:.1f}°")
            except ValueError:
                self.degree_vars[spec.define].set("数值无效")

    def _update_dirty_state(self) -> None:
        if not self.loaded_values:
            return
        dirty = False
        for spec in PARAMETERS:
            try:
                value = int(self.value_vars[spec.define].get().strip())
            except ValueError:
                dirty = True
                break
            if value != self.loaded_values.get(spec.define):
                dirty = True
                break
        self.dirty_var.set("● 有尚未保存的参数" if dirty else "参数与源码一致")

    def _nudge(self, spec: ParameterSpec, delta: int) -> None:
        try:
            value = int(self.value_vars[spec.define].get().strip())
        except ValueError:
            value = self.loaded_values.get(spec.define, spec.minimum)
        value = max(spec.minimum, min(spec.maximum, value + delta))
        self.value_vars[spec.define].set(str(value))

    def _update_servo2_manual_degree(self) -> None:
        try:
            pulse = int(self.servo2_manual_var.get().strip())
        except ValueError:
            self.servo2_manual_degree_var.set("数值无效")
            return
        degrees = approximate_degrees("servo2", pulse)
        self.servo2_manual_degree_var.set(f"us  ≈ {degrees:.1f}°")

    def _nudge_servo2_manual(self, delta: int) -> None:
        try:
            value = int(self.servo2_manual_var.get().strip())
        except ValueError:
            value = self.loaded_values.get("SERVO2_CLAMP_PULSE_US", SERVO_TUNING_MIN_PULSE_US)
        value = max(SERVO_TUNING_MIN_PULSE_US, min(SERVO_TUNING_MAX_PULSE_US, value + delta))
        self.servo2_manual_var.set(str(value))

    def _test_servo2_manual(self) -> None:
        if not self._safety_check("servo2", needs_lift=False):
            return
        try:
            value = int(self.servo2_manual_var.get().strip())
        except ValueError:
            messagebox.showerror("参数错误", "夹爪手动脉宽必须是整数。")
            return
        if not SERVO_TUNING_MIN_PULSE_US <= value <= SERVO_TUNING_MAX_PULSE_US:
            messagebox.showerror(
                "参数错误",
                f"夹爪手动试动范围：{SERVO_TUNING_MIN_PULSE_US}–{SERVO_TUNING_MAX_PULSE_US} us",
            )
            return
        frame = make_frame(TUNING_MODE_SERVO2, value)
        detail = (
            f"Servo2 夹爪自由试动 → {value} us"
            f"（约 {approximate_degrees('servo2', value):.1f}°）\n"
            f"mode {TUNING_MODE_SERVO2}\n{hex_bytes(frame)}"
        )
        if not messagebox.askyesno("确认夹爪试动", detail + "\n\n此数值不会写入正式任务参数。确认执行？"):
            return
        self._send_frame(frame, f"Servo2 自由试动={value}us", timeout_s=8.0)

    def _changes(self, values: dict[str, int]) -> list[tuple[ParameterSpec, int, int]]:
        changes = []
        for spec in PARAMETERS:
            old = self.loaded_values.get(spec.define)
            new = values[spec.define]
            if old is not None and old != new:
                changes.append((spec, old, new))
        return changes

    def _preview_changes(self) -> None:
        values = self._validated_values()
        if values is None:
            return
        changes = self._changes(values)
        if not changes:
            messagebox.showinfo("无修改", "当前参数与已读取的源码一致。")
            return
        window = tk.Toplevel(self)
        window.title("参数修改预览")
        window.geometry("760x520")
        text = tk.Text(window, wrap="word", font=("Cascadia Mono", 10), padx=10, pady=10)
        text.pack(fill="both", expand=True)
        for spec, old, new in changes:
            text.insert("end", f"{spec.label}\n  {spec.define}: {old} -> {new} {spec.unit}\n\n")
        text.configure(state="disabled")

    def _save_source(self) -> bool:
        values = self._validated_values()
        if values is None:
            return False
        changes = self._changes(values)
        if not changes:
            messagebox.showinfo("无需保存", "参数已经与源码一致。")
            return True
        summary = "\n".join(f"{spec.label}: {old} → {new} {spec.unit}" for spec, old, new in changes[:12])
        if len(changes) > 12:
            summary += f"\n……另有 {len(changes) - 12} 项"
        if not messagebox.askyesno(
            "确认写入当前 .c",
            f"将只修改以下参数的数字：\n\n{summary}\n\n是否继续？",
        ):
            return False
        try:
            self.loaded_hash = replace_source_parameters(
                self.source_path, self.loaded_hash, values
            )
        except Exception as exc:
            messagebox.showerror("保存失败", str(exc))
            return False
        self.loaded_values = dict(values)
        self._update_dirty_state()
        self._append_log(f"已保存 {len(changes)} 项参数到 {self.source_path}", "info")
        messagebox.showinfo("保存完成", "参数已精确写回当前 .c；尚未编译或烧录。")
        return True

    def _export_profile(self) -> None:
        values = self._validated_values()
        if values is None:
            return
        path = filedialog.asksaveasfilename(
            title="导出调参方案",
            defaultextension=".json",
            filetypes=(("JSON 参数方案", "*.json"),),
            initialfile=f"hardware_tuning_{datetime.now():%Y%m%d_%H%M%S}.json",
        )
        if not path:
            return
        payload = {
            "format": 1,
            "source": str(self.source_path),
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "parameters": values,
        }
        Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self._append_log(f"已导出参数方案：{path}", "info")

    def _import_profile(self) -> None:
        path = filedialog.askopenfilename(
            title="导入调参方案", filetypes=(("JSON 参数方案", "*.json"),)
        )
        if not path:
            return
        try:
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
            values = payload["parameters"]
            missing = [spec.define for spec in PARAMETERS if spec.define not in values]
            if missing:
                raise ValueError("方案缺少参数：" + ", ".join(missing[:5]))
            for spec in PARAMETERS:
                self.value_vars[spec.define].set(str(int(values[spec.define])))
            if self._validated_values() is None:
                raise ValueError("方案中存在超出安全范围的数值")
        except Exception as exc:
            messagebox.showerror("导入失败", str(exc))
            return
        self._append_log(f"已载入参数方案：{path}；尚未写入源码或发送动作。", "info")

    def _safety_check(self, actuator: str | None, needs_lift: bool) -> bool:
        if not self.connected:
            messagebox.showwarning("尚未连接", f"请先连接 {SERIAL_PORT}。")
            return False
        if self.busy or self.build_busy:
            messagebox.showinfo("操作进行中", "请等待上一项操作结束。")
            return False
        if not self.action_armed_var.get():
            messagebox.showwarning("机械动作未授权", "请先勾选“已清空运动范围”。")
            return False
        if needs_lift or actuator == "lift":
            if not self.lift_synced_var.get():
                messagebox.showwarning(
                    "升降高度未同步",
                    "请先人工把升降机构放到最低点，确认软件 0 mm 与实际位置同步后再勾选。",
                )
                return False
        if actuator == "servo3" and not self.turntable_safe_var.get():
            messagebox.showwarning(
                "转台区域未确认", "请先确认升降/机械臂处于安全高度且转台范围无干涉。"
            )
            return False
        return True

    def _test_parameter(self, spec: ParameterSpec) -> None:
        if not self._safety_check(spec.actuator, needs_lift=False):
            return
        try:
            value = int(self.value_vars[spec.define].get().strip())
        except ValueError:
            messagebox.showerror("参数错误", f"{spec.label} 必须是整数。")
            return
        if not spec.minimum <= value <= spec.maximum:
            messagebox.showerror("参数错误", f"允许范围：{spec.minimum}–{spec.maximum} {spec.unit}")
            return
        modes = {
            "servo1": TUNING_MODE_SERVO1,
            "servo2": TUNING_MODE_SERVO2,
            "servo3": TUNING_MODE_SERVO3,
            "lift": TUNING_MODE_LIFT,
        }
        mode = modes[spec.actuator]
        frame = make_frame(mode, value)
        detail = f"{spec.label} → {value} {spec.unit}\nmode {mode}\n{hex_bytes(frame)}"
        if not messagebox.askyesno("确认单项试动", detail + "\n\n确认机械范围安全并执行？"):
            return
        self._send_frame(frame, f"{spec.label}={value}{spec.unit}", timeout_s=8.0)

    def _send_mode_action(self, mode: int, label: str, needs_lift: bool) -> None:
        actuator = "servo3" if mode == 86 else None
        if not self._safety_check(actuator, needs_lift=needs_lift):
            return
        frame = make_frame(mode)
        if not messagebox.askyesno(
            "确认执行机械动作",
            f"{label}\nmode {mode}\n{hex_bytes(frame)}\n\n确认执行？",
        ):
            return
        self._send_frame(
            frame,
            f"{label} (mode {mode})",
            timeout_s=60.0,
        )

    def _send_frame(self, frame: bytes, label: str, timeout_s: float) -> None:
        if self.worker is None:
            return
        self.busy = True
        self.status_var.set(f"正在执行：{label}")
        self.worker.request("send", (frame, label, timeout_s))

    def _start_build(self, export_bin: bool, flash: bool) -> None:
        if self.build_busy or self.busy:
            messagebox.showinfo("操作进行中", "请等待上一项操作结束。")
            return
        values = self._validated_values()
        if values is None:
            return
        if self._changes(values):
            if not messagebox.askyesno("存在未保存参数", "先把当前界面参数保存到源码再继续构建？"):
                return
            if not self._save_source():
                return
        if flash:
            if self.connected:
                messagebox.showwarning("请先断开串口", f"烧录前请点击“断开”，释放 {SERIAL_PORT}。")
                return
            ports = [] if list_ports is None else [item.device.upper() for item in list_ports.comports()]
            if SERIAL_PORT not in ports:
                messagebox.showwarning(f"{SERIAL_PORT} 不存在", "请重新插拔 USB；不会改用其他串口。")
                return
            if not messagebox.askyesno(
                f"确认烧录 {SERIAL_PORT}",
                f"将编译当前工程、导出新 BIN，并擦除/写入 {SERIAL_PORT}。\n烧录后单片机会复位，请确认机械范围安全。\n\n确认继续？",
            ):
                return
        self.build_busy = True
        self.status_var.set("正在构建…")
        requested_bin_name = self.bin_name_var.get().strip()
        threading.Thread(
            target=self._build_worker,
            args=(export_bin, flash, requested_bin_name),
            daemon=True,
        ).start()

    def _run_process(self, command: list[str], label: str) -> tuple[int, str]:
        self.events.put({"event": "build_log", "message": f"{label}: {' '.join(command)}"})
        process = subprocess.run(
            command,
            cwd=DEFAULT_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="gbk",
            errors="replace",
            check=False,
        )
        output = process.stdout or ""
        if output.strip():
            self.events.put({"event": "build_log", "message": output.rstrip()})
        return process.returncode, output

    def _build_worker(
        self, export_bin: bool, flash: bool, requested_bin_name: str
    ) -> None:
        try:
            OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
            LISTING_DIR.mkdir(parents=True, exist_ok=True)
            if KEIL_PATH is None or not KEIL_PATH.exists():
                raise FileNotFoundError(
                    "找不到 Keil UV4。请安装 MDK-ARM，或设置 KEIL_ROOT/KEIL_UV4 环境变量。"
                )
            if not PROJECT_PATH.exists():
                raise FileNotFoundError(f"找不到 Keil 工程：{PROJECT_PATH}")
            code, _ = self._run_process(
                [str(KEIL_PATH), "-b", str(PROJECT_PATH), "-j0"], "Keil 编译"
            )
            if code != 0:
                raise RuntimeError(f"Keil 进程返回 {code}")
            log_text = BUILD_LOG_PATH.read_bytes().decode("gbk", errors="replace")
            if "0 Error(s), 0 Warning(s)" not in log_text:
                match = re.search(r"\d+ Error\(s\), \d+ Warning\(s\)", log_text)
                raise RuntimeError("构建日志未通过：" + (match.group(0) if match else "未找到结果行"))
            self.events.put({"event": "build_log", "message": "构建日志确认：0 Error(s), 0 Warning(s)"})

            bin_path: Path | None = None
            if export_bin:
                if FROMELF_PATH is None or not FROMELF_PATH.exists():
                    raise FileNotFoundError(
                        "找不到 fromelf。请设置 KEIL_ROOT 或 KEIL_FROMELF 环境变量。"
                    )
                if requested_bin_name:
                    safe_name = Path(requested_bin_name).name
                    if not safe_name.lower().endswith(".bin"):
                        safe_name += ".bin"
                else:
                    version = read_firmware_version(self.source_path)
                    safe_name = f"chassis_hardware_tuning_v{version}_{datetime.now():%Y%m%d_%H%M%S}.bin"
                bin_path = OUTPUT_DIR / safe_name
                if bin_path.exists():
                    bin_path = bin_path.with_name(
                        f"{bin_path.stem}_{datetime.now():%Y%m%d_%H%M%S}{bin_path.suffix}"
                    )
                code, _ = self._run_process(
                    [str(FROMELF_PATH), "--bin", "--output", str(bin_path), str(AXF_PATH)],
                    "导出 BIN",
                )
                if code != 0 or not bin_path.exists():
                    raise RuntimeError("BIN 导出失败")
                self.events.put({"event": "build_log", "message": f"已导出：{bin_path}"})

            if flash:
                if bin_path is None:
                    raise RuntimeError("内部错误：烧录前没有 BIN")
                if not PYTHON_PATH.exists():
                    raise FileNotFoundError(f"找不到当前 Python：{PYTHON_PATH}")
                ports = [] if list_ports is None else [item.device.upper() for item in list_ports.comports()]
                self.events.put({"event": "build_log", "message": "烧录前串口扫描：" + (", ".join(ports) if ports else "无")})
                if SERIAL_PORT not in ports:
                    raise RuntimeError(f"{SERIAL_PORT} 不存在，请重新插拔 USB；未尝试其他串口")
                loader_code = (
                    "import sys; "
                    "from stm32loader.main import main; main(*sys.argv[1:])"
                )
                code, output = self._run_process(
                    [
                        str(PYTHON_PATH),
                        "-c",
                        loader_code,
                        "-p",
                        SERIAL_PORT,
                        "-b",
                        str(SERIAL_BAUD),
                        "-f",
                        "F1",
                        "-e",
                        "-w",
                        "-v",
                        "-g",
                        "0x08000000",
                        "-R",
                        "-B",
                        "-V",
                        str(bin_path),
                    ],
                    f"烧录 {SERIAL_PORT}",
                )
                if code != 0 or "Verification OK" not in output:
                    raise RuntimeError("未看到 Verification OK，不能判定烧录成功")
                self.events.put({"event": "flash_verified", "bin_path": str(bin_path)})
            else:
                self.events.put({"event": "build_success", "bin_path": str(bin_path) if bin_path else ""})
        except Exception as exc:
            self.events.put({"event": "build_error", "message": str(exc)})
        finally:
            self.events.put({"event": "build_done"})

    def _poll_events(self) -> None:
        try:
            while True:
                event = self.events.get_nowait()
                kind = event["event"]
                if kind == "connected":
                    self.connected = True
                    self.connect_button.configure(text="断开", state="normal")
                    self.connection_var.set(f"已连接 {SERIAL_PORT}")
                    self.status_var.set("串口已连接；尚未发送动作")
                    self._append_log(f"已连接 {SERIAL_PORT} @ {SERIAL_BAUD}。", "info")
                elif kind == "disconnected":
                    self.connected = False
                    self.busy = False
                    self.connect_button.configure(text="连接", state="normal")
                    self.connection_var.set("未连接")
                    self.status_var.set("串口已断开")
                    self._append_log(f"{SERIAL_PORT} 已断开。", "warn")
                elif kind == "tx":
                    self._append_log(f"TX [{event['label']}]: {hex_bytes(event['frame'])}", "tx")
                elif kind == "rx":
                    status = int(event["status"])
                    elapsed = event["elapsed"]
                    suffix = "" if elapsed is None else f"，{float(elapsed) * 1000:.1f} ms"
                    name = STATUS_NAMES.get(status, f"状态 0x{status:02X}")
                    self._append_log(f"RX: {hex_bytes(event['frame'])} ({name}{suffix})", "rx" if status in (1, 2, 3) else "warn")
                    self.status_var.set(name + suffix)
                elif kind == "noise":
                    data = event["data"]
                    if data:
                        self._append_log(f"RX 其他数据: {hex_bytes(data)}", "warn")
                elif kind == "timeout":
                    self._append_log(f"超时 [{event['label']}]：{float(event['elapsed']):.1f}s 未收到 ACK；没有自动重发。", "error")
                    self.status_var.set("等待 ACK 超时")
                elif kind == "operation_done":
                    self.busy = False
                elif kind == "serial_error":
                    self.busy = False
                    self.connect_button.configure(state="normal")
                    self.status_var.set("串口错误")
                    self._append_log(str(event["message"]), "error")
                elif kind == "build_log":
                    self._append_log(str(event["message"]), "info")
                elif kind == "build_success":
                    path = str(event["bin_path"])
                    self.status_var.set("构建与导出完成" if path else "编译检查通过")
                    messagebox.showinfo("构建完成", "0 Error(s), 0 Warning(s)" + (f"\nBIN：{path}" if path else ""))
                elif kind == "flash_verified":
                    self.action_armed_var.set(False)
                    self.lift_synced_var.set(False)
                    self.turntable_safe_var.set(False)
                    self.status_var.set("烧录验证成功")
                    messagebox.showinfo("烧录成功", f"Verification OK\nBIN：{event['bin_path']}")
                elif kind == "build_error":
                    self.status_var.set("构建/烧录失败")
                    self._append_log(str(event["message"]), "error")
                    messagebox.showerror("构建/烧录失败", str(event["message"]))
                elif kind == "build_done":
                    self.build_busy = False
        except queue.Empty:
            pass
        self.after(50, self._poll_events)

    def _append_log(self, text: str, tag: str) -> None:
        if not hasattr(self, "log"):
            return
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.configure(state="normal")
        for line in str(text).splitlines() or [""]:
            self.log.insert("end", f"[{timestamp}] {line}\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _on_close(self) -> None:
        if self.busy or self.build_busy:
            if not messagebox.askyesno("操作仍在进行", "当前操作尚未结束，仍要关闭界面吗？"):
                return
        if self.worker is not None:
            self.worker.request("stop")
        self.destroy()


def self_test(source_path: Path) -> int:
    values, source_hash = parse_source_parameters(source_path)
    assert len(values) == len(PARAMETERS)
    assert read_firmware_version(source_path) >= 130
    assert make_frame(TUNING_MODE_SERVO1, 946) == bytes(
        [0xFF, 80, 0x03, 0xB2, 0, 0, 0, 0, 0, 0xFE]
    )
    assert make_frame(TUNING_MODE_SERVO2, SERVO_TUNING_MAX_PULSE_US) == bytes(
        [0xFF, 81, 0x09, 0xC4, 0, 0, 0, 0, 0, 0xFE]
    )
    assert make_frame(72) == bytes([0xFF, 72, 0, 0, 0, 0, 0, 0, 0, 0xFE])
    for spec in PARAMETERS:
        assert spec.minimum <= values[spec.define] <= spec.maximum, spec.define
    print(f"SELF-TEST OK: {len(values)} parameters, source SHA-256 {source_hash}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=APP_TITLE)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if args.self_test:
        return self_test(args.source.resolve())
    app = HardwareTuningApp(args.source)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
