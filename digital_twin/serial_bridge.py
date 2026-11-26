"""Thread-safe STM32 UART bridge with pyserial, Win32 ctypes fallback, and simulation mode."""

from __future__ import annotations

import ctypes
import queue
import random
import sys
import threading
import time
from ctypes import wintypes
from typing import Callable, Optional

from .pneumatics import PneumaticChamberModel, PneumaticParameters
from .protocol import EventMessage, TelemetrySample, parse_stm32_line

try:
    import serial  # type: ignore
    import serial.tools.list_ports  # type: ignore

    HAS_PYSERIAL = True
except ImportError:
    serial = None  # type: ignore
    HAS_PYSERIAL = False


def list_serial_ports() -> list[str]:
    """Return available serial port names on the host."""
    if HAS_PYSERIAL:
        return [p.device for p in serial.tools.list_ports.comports()]

    if sys.platform == "win32":
        import winreg

        ports: list[str] = []
        try:
            with winreg.OpenKey(
                winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM"
            ) as key:
                idx = 0
                while True:
                    try:
                        _, value, _ = winreg.EnumValue(key, idx)
                        ports.append(str(value))
                        idx += 1
                    except OSError:
                        break
        except OSError:
            pass
        return sorted(ports)

    return []


class _Win32SerialPort:
    """Minimal pure-Python Win32 serial port wrapper when pyserial is not installed."""

    GENERIC_READ = 0x80000000
    GENERIC_WRITE = 0x40000000
    OPEN_EXISTING = 3
    INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

    class _DCB(ctypes.Structure):
        _fields_ = [
            ("DCBlength", wintypes.DWORD),
            ("BaudRate", wintypes.DWORD),
            ("fBinary", wintypes.DWORD, 1),
            ("fParity", wintypes.DWORD, 1),
            ("fOutxCtsFlow", wintypes.DWORD, 1),
            ("fOutxDsrFlow", wintypes.DWORD, 1),
            ("fDtrControl", wintypes.DWORD, 2),
            ("fDsrSensitivity", wintypes.DWORD, 1),
            ("fTXContinueOnXoff", wintypes.DWORD, 1),
            ("fOutX", wintypes.DWORD, 1),
            ("fInX", wintypes.DWORD, 1),
            ("fErrorChar", wintypes.DWORD, 1),
            ("fNull", wintypes.DWORD, 1),
            ("fRtsControl", wintypes.DWORD, 2),
            ("fAbortOnError", wintypes.DWORD, 1),
            ("fDummy2", wintypes.DWORD, 17),
            ("wReserved", wintypes.WORD),
            ("XonLim", wintypes.WORD),
            ("XoffLim", wintypes.WORD),
            ("ByteSize", wintypes.BYTE),
            ("Parity", wintypes.BYTE),
            ("StopBits", wintypes.BYTE),
            ("XonChar", ctypes.c_char),
            ("XoffChar", ctypes.c_char),
            ("ErrorChar", ctypes.c_char),
            ("EofChar", ctypes.c_char),
            ("EvtChar", ctypes.c_char),
            ("wReserved1", wintypes.WORD),
        ]

    class _COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout", wintypes.DWORD),
            ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
            ("ReadTotalTimeoutConstant", wintypes.DWORD),
            ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
            ("WriteTotalTimeoutConstant", wintypes.DWORD),
        ]

    def __init__(self, port: str, baudrate: int = 115200, timeout_ms: int = 50) -> None:
        self._k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self._k32.CreateFileW.argtypes = [
            wintypes.LPCWSTR,
            wintypes.DWORD,
            wintypes.DWORD,
            wintypes.LPVOID,
            wintypes.DWORD,
            wintypes.DWORD,
            wintypes.HANDLE,
        ]
        self._k32.CreateFileW.restype = wintypes.HANDLE
        self._k32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(self._DCB)]
        self._k32.GetCommState.restype = wintypes.BOOL
        self._k32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(self._DCB)]
        self._k32.SetCommState.restype = wintypes.BOOL
        self._k32.SetCommTimeouts.argtypes = [wintypes.HANDLE, ctypes.POINTER(self._COMMTIMEOUTS)]
        self._k32.SetCommTimeouts.restype = wintypes.BOOL
        self._k32.ReadFile.argtypes = [
            wintypes.HANDLE,
            wintypes.LPVOID,
            wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD),
            wintypes.LPVOID,
        ]
        self._k32.ReadFile.restype = wintypes.BOOL
        self._k32.WriteFile.argtypes = [
            wintypes.HANDLE,
            wintypes.LPCVOID,
            wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD),
            wintypes.LPVOID,
        ]
        self._k32.WriteFile.restype = wintypes.BOOL
        self._k32.CloseHandle.argtypes = [wintypes.HANDLE]
        self._k32.CloseHandle.restype = wintypes.BOOL
        self._k32.PurgeComm.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        self._k32.PurgeComm.restype = wintypes.BOOL

        device_path = port if port.startswith("\\\\.\\") else f"\\\\.\\{port}"
        self._handle = self._k32.CreateFileW(
            device_path,
            self.GENERIC_READ | self.GENERIC_WRITE,
            0,
            None,
            self.OPEN_EXISTING,
            0,
            None,
        )
        if not self._handle or self._handle == self.INVALID_HANDLE_VALUE:
            raise ctypes.WinError(ctypes.get_last_error())

        dcb = self._DCB()
        dcb.DCBlength = ctypes.sizeof(self._DCB)
        if not self._k32.GetCommState(self._handle, ctypes.byref(dcb)):
            self.close()
            raise ctypes.WinError(ctypes.get_last_error())

        dcb.BaudRate = baudrate
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0  # 1 stop bit
        dcb.fBinary = 1
        dcb.fParity = 0
        dcb.fDtrControl = 1
        dcb.fRtsControl = 1
        if not self._k32.SetCommState(self._handle, ctypes.byref(dcb)):
            self.close()
            raise ctypes.WinError(ctypes.get_last_error())

        timeouts = self._COMMTIMEOUTS(
            ReadIntervalTimeout=10,
            ReadTotalTimeoutMultiplier=0,
            ReadTotalTimeoutConstant=timeout_ms,
            WriteTotalTimeoutMultiplier=0,
            WriteTotalTimeoutConstant=100,
        )
        self._k32.SetCommTimeouts(self._handle, ctypes.byref(timeouts))
        self._k32.PurgeComm(self._handle, 0x0008 | 0x0004)  # PURGE_RXCLEAR | PURGE_TXCLEAR
        self._rx_buf = bytearray()

    def readline(self) -> bytes:
        while True:
            nl_idx = self._rx_buf.find(b"\n")
            if nl_idx >= 0:
                line = bytes(self._rx_buf[: nl_idx + 1])
                del self._rx_buf[: nl_idx + 1]
                return line

            chunk = ctypes.create_string_buffer(256)
            bytes_read = wintypes.DWORD(0)
            ok = self._k32.ReadFile(self._handle, chunk, 256, ctypes.byref(bytes_read), None)
            if not ok:
                raise ctypes.WinError(ctypes.get_last_error())
            if bytes_read.value == 0:
                return b""
            self._rx_buf.extend(chunk.raw[: bytes_read.value])

    def write(self, data: bytes) -> int:
        written = wintypes.DWORD(0)
        buf = ctypes.create_string_buffer(data, len(data))
        ok = self._k32.WriteFile(self._handle, buf, len(data), ctypes.byref(written), None)
        if not ok:
            raise ctypes.WinError(ctypes.get_last_error())
        return int(written.value)

    def close(self) -> None:
        if getattr(self, "_handle", None) not in (None, self.INVALID_HANDLE_VALUE):
            self._k32.CloseHandle(self._handle)
            self._handle = self.INVALID_HANDLE_VALUE


class SimulatedSTM32Transport:
    """Software twin of the STM32U545 valve + T150PGSA3XX SPI firmware at 50 Hz."""

    def __init__(self, params: PneumaticParameters | None = None, seed: int = 42) -> None:
        self._params = params or PneumaticParameters()
        # Slightly mismatched plant vs nominal twin so sync RMSE is realistic & measurable
        plant_params = PneumaticParameters(
            supply_pressure_kpa=self._params.supply_pressure_kpa * 0.98,
            tau_charge_s=self._params.tau_charge_s * 1.08,
            tau_discharge_s=self._params.tau_discharge_s * 0.94,
            tau_leak_s=self._params.tau_leak_s,
        )
        self._chamber = PneumaticChamberModel(plant_params)
        self._rng = random.Random(seed)
        self._rx_lines: queue.Queue[bytes] = queue.Queue()
        self._lock = threading.Lock()
        self._closed = False

        self._pwm1 = 0
        self._pwm2 = 0
        self._auto_target = False
        self._target_kpa = 0.0
        self._max_safe_kpa = 250.0
        self._s0_offset = 0.0
        self._s1_offset = 0.0
        self._overpressure_tripped = False

        self._rx_lines.put(b"SYSTEM,READY\r\n")
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        period = 0.02  # 50 Hz matching PRESSURE_SAMPLE_PERIOD_MS = 20U
        next_tick = time.perf_counter()
        while not self._closed:
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(min(0.005, next_tick - now))
                continue
            next_tick += period

            with self._lock:
                self._chamber.step(period, self._pwm1 / 999.0, self._pwm2 / 999.0)
                raw_p1 = self._chamber.pressure_kpa + self._rng.gauss(0.0, 0.25)
                raw_p0 = self._params.supply_pressure_kpa + self._rng.gauss(0.0, 0.35)
                p0 = raw_p0 - self._s0_offset
                p1 = raw_p1 - self._s1_offset

                if self._auto_target:
                    if p1 < self._target_kpa - 3.0:
                        self._pwm1, self._pwm2 = 999, 0
                    elif p1 > self._target_kpa + 3.0:
                        self._pwm1, self._pwm2 = 0, 999
                    else:
                        self._pwm1, self._pwm2 = 0, 0

                if p1 > self._max_safe_kpa:
                    self._auto_target = False
                    self._pwm1, self._pwm2 = 0, 999
                    if not self._overpressure_tripped:
                        self._overpressure_tripped = True
                        self._rx_lines.put(f"WARN,OVERPRESSURE,{p1:.2f}\r\n".encode("ascii"))
                elif self._overpressure_tripped and p1 <= self._max_safe_kpa - 3.0:
                    self._overpressure_tripped = False
                    self._pwm2 = 0

                v1 = 1 if self._pwm1 > 0 else 0
                v2 = 1 if self._pwm2 > 0 else 0
                msg = (
                    f"DATA,S0:{p0:.4f},S1:{p1:.4f},V1:{v1},V2:{v2},"
                    f"PWM1:{self._pwm1},PWM2:{self._pwm2},T1:24.80\r\n"
                ).encode("ascii")

            self._rx_lines.put(msg)

    def readline(self) -> bytes:
        try:
            return self._rx_lines.get(timeout=0.05)
        except queue.Empty:
            return b""

    def write(self, data: bytes) -> int:
        # Simulate slight UART/MCU round-trip delay (~2-4 ms)
        time.sleep(0.002)
        text = data.decode("ascii", errors="ignore")
        for line in text.splitlines():
            cmd = line.strip()
            if not cmd:
                continue
            self._handle_command(cmd)
        return len(data)

    def _handle_command(self, cmd: str) -> None:
        with self._lock:
            if cmd == "PING":
                self._rx_lines.put(b"PONG\r\n")
            elif cmd == "VALVE,1,ON":
                self._auto_target = False
                self._pwm1 = 999
                self._rx_lines.put(b"ACK,VALVE,1,ON\r\n")
            elif cmd == "VALVE,1,OFF":
                self._auto_target = False
                self._pwm1 = 0
                self._rx_lines.put(b"ACK,VALVE,1,OFF\r\n")
            elif cmd == "VALVE,2,ON":
                self._auto_target = False
                self._pwm2 = 999
                self._rx_lines.put(b"ACK,VALVE,2,ON\r\n")
            elif cmd == "VALVE,2,OFF":
                self._auto_target = False
                self._pwm2 = 0
                self._rx_lines.put(b"ACK,VALVE,2,OFF\r\n")
            elif cmd.startswith("PWM,"):
                parts = cmd.split(",")
                if len(parts) == 3 and parts[1] in ("1", "2"):
                    duty = max(0, min(999, int(parts[2])))
                    self._auto_target = False
                    if parts[1] == "1":
                        self._pwm1 = duty
                    else:
                        self._pwm2 = duty
                    self._rx_lines.put(f"ACK,PWM,{parts[1]},{duty}\r\n".encode("ascii"))
            elif cmd == "TARGET,OFF":
                self._auto_target = False
                self._pwm1 = 0
                self._pwm2 = 0
                self._rx_lines.put(b"ACK,TARGET,OFF\r\n")
            elif cmd.startswith("TARGET,"):
                req = max(0.0, min(self._max_safe_kpa, float(cmd.split(",", 1)[1])))
                self._target_kpa = req
                self._auto_target = True
                self._rx_lines.put(f"ACK,TARGET,{req:.2f}\r\n".encode("ascii"))
            elif cmd == "ZERO":
                self._s1_offset = self._chamber.pressure_kpa
                self._rx_lines.put(
                    f"ACK,ZERO,{self._s0_offset:.4f},{self._s1_offset:.4f}\r\n".encode("ascii")
                )
            elif cmd == "ZERO,RESET":
                self._s0_offset = 0.0
                self._s1_offset = 0.0
                self._rx_lines.put(b"ACK,ZERO,RESET\r\n")
            elif cmd.startswith("MAXP,"):
                lim = max(1.0, min(1000.0, float(cmd.split(",", 1)[1])))
                self._max_safe_kpa = lim
                self._rx_lines.put(f"ACK,MAXP,{lim:.2f}\r\n".encode("ascii"))
            elif cmd == "STATUS":
                v1 = 1 if self._pwm1 > 0 else 0
                v2 = 1 if self._pwm2 > 0 else 0
                self._rx_lines.put(
                    f"STATUS,VALVE1={v1},VALVE2={v2},PWM1={self._pwm1},PWM2={self._pwm2},"
                    f"TARGET={self._target_kpa:.2f},AUTO={int(self._auto_target)},"
                    f"MAXP={self._max_safe_kpa:.2f}\r\n".encode("ascii")
                )
            else:
                self._rx_lines.put(f"ERROR,UNKNOWN_COMMAND,{cmd}\r\n".encode("ascii"))

    def close(self) -> None:
        self._closed = True
        self._thread.join(timeout=0.5)


class STM32Bridge:
    """Asynchronous reader/writer managing the STM32 serial link or simulated hardware."""

    def __init__(
        self,
        port: str = "SIM",
        baudrate: int = 115200,
        params: PneumaticParameters | None = None,
        on_telemetry: Optional[Callable[[TelemetrySample], None]] = None,
        on_event: Optional[Callable[[EventMessage], None]] = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self._params = params or PneumaticParameters()
        self._on_telemetry = on_telemetry
        self._on_event = on_event

        self._transport = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._write_lock = threading.Lock()
        self._ping_sent_ts: Optional[float] = None
        self.last_rtt_ms: Optional[float] = None

    @property
    def is_connected(self) -> bool:
        return self._running and self._transport is not None

    def start(self) -> None:
        if self._running:
            return
        if self.port.upper() == "SIM":
            self._transport = SimulatedSTM32Transport(params=self._params)
        elif HAS_PYSERIAL:
            self._transport = serial.Serial(self.port, baudrate=self.baudrate, timeout=0.05)
            self._transport.reset_input_buffer()
        elif sys.platform == "win32":
            self._transport = _Win32SerialPort(self.port, baudrate=self.baudrate, timeout_ms=50)
        else:
            raise RuntimeError("pyserial is required on non-Windows platforms")

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._transport is not None:
            self._transport.close()
            self._transport = None

    def send_command(self, command: str) -> None:
        if not self._transport:
            return
        payload = command if command.endswith("\n") else f"{command}\n"
        with self._write_lock:
            if payload.strip() == "PING":
                self._ping_sent_ts = time.perf_counter()
            self._transport.write(payload.encode("ascii"))

    def ping(self) -> None:
        self.send_command("PING\n")

    def _read_loop(self) -> None:
        while self._running and self._transport is not None:
            try:
                raw = self._transport.readline()
            except Exception as exc:
                if self._on_event:
                    self._on_event(
                        EventMessage(
                            timestamp_s=time.perf_counter(),
                            kind="LINK_ERROR",
                            raw=f"LINK_ERROR,{exc}",
                        )
                    )
                self._running = False
                break

            if not raw:
                continue

            now = time.perf_counter()
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue

            sample, event = parse_stm32_line(line, now)
            if sample is not None and self._on_telemetry:
                self._on_telemetry(sample)
            elif event is not None:
                if event.kind == "PONG" and self._ping_sent_ts is not None:
                    self.last_rtt_ms = (now - self._ping_sent_ts) * 1000.0
                    self._ping_sent_ts = None
                if self._on_event:
                    self._on_event(event)
