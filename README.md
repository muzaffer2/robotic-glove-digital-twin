# Textile-Based Pneumatic Robotic Glove — Digital Twin & STM32U545 Firmware

End-to-end physical–virtual Digital Twin system combining the **STM32U545** pneumatic valve & **Honeywell ABP2 (`T150PGSA3XX`)** SPI pressure-sensing firmware with a **Python + PyBullet** real-time 3D virtual glove, lumped-parameter pneumatic chamber model, and synchronization evaluator.

## Hardware Mapping (`STM32U545` + Breadboard)

- **Actuation (`TIM2`, 1 kHz PWM):**
  - `PA0` (`TIM2_CH1`): Valve 1 MOSFET (Inflate) + `LED2` (`PA5`)
  - `PA1` (`TIM2_CH2`): Valve 2 MOSFET (Exhaust/Deflate)
- **Pressure Sensing (`SPI2` @ 625 kHz, 50 Hz):**
  - `PB13` (`SCK`), `PB14` (`MISO`), `PB15` (`MOSI`)
  - `PA2`: Sensor 0 (`S0`) Chip Select
  - `PA3`: Sensor 1 (`S1`, pneumatic tube) Chip Select
  - Sensor calibration: Honeywell ABP2 **`T150PGSA3XX`** (`0–150 psi` Gage = `0.0–1034.2136 kPa`, 10%–90% of 24-bit counts, 3.3 V)
- **Telemetry & Control (`USART1` DMA @ 115200 baud, ST-LINK VCP):**
  - `PA9` (TX), `PA10` (RX)

## Serial Protocol (115200 baud, `\n` terminated)

- **Telemetry (50 Hz):** `DATA,S0:<kpa>,S1:<kpa>,V1:<0|1>,V2:<0|1>,PWM1:<0-999>,PWM2:<0-999>,T1:<degC>`
- **Commands:**
  - `PING` → `PONG`
  - `STATUS` → `STATUS,VALVE1=...,VALVE2=...,PWM1=...,PWM2=...,TARGET=...,AUTO=...,MAXP=...`
  - `VALVE,<1|2>,<ON|OFF>` → switch valve ON/OFF (`ACK,VALVE,...`)
  - `PWM,<1|2>,<0-999>` → set proportional PWM duty (`ACK,PWM,...`)
  - `TARGET,<kpa>` / `TARGET,OFF` → on-board closed-loop pressure control (hysteresis ±3 kPa)
  - `ZERO` / `ZERO,RESET` → tare atmospheric zero offset on-board
  - `MAXP,<kpa>` → set overpressure safety cutoff (default `250.0 kPa`, max `1000.0 kPa`)

## Running the Python Digital Twin

Install dependencies (optional on Windows since `ctypes` Win32 serial fallback is built in, though `pyserial` is recommended):

```bash
pip install -r requirements.txt
```

### 1. Simulation Mode (No Hardware Required)

```bash
python -m digital_twin.app --port SIM
```

### 2. Live STM32 Hardware Mode

Connect the Nucleo board via USB, check the ST-LINK Virtual COM Port (e.g., `COM3`), and run:

```bash
python -m digital_twin.app --port COM3
```

Add `--no-pybullet` to run only the Tkinter telemetry/2D-finger dashboard without opening the PyBullet 3D window.

### 3. Synchronization Evaluation & CSV Logging

Run a scripted inflation → hold → closed-loop curl → vent benchmark and save timestamped telemetry to `logs/`:

```bash
python -m digital_twin.evaluate_sync --port SIM --duration 8
# or against physical hardware:
python -m digital_twin.evaluate_sync --port COM3 --duration 10
```

### 4. Long-Term Actuator Endurance & Drift Test

Run repeated `inflate → hold (leak test) → deflate → rest` cycles with safety watchdogs (overpressure limit, sensor disconnect abort, and auto-vent on exit), exporting per-cycle fatigue/drift metrics (`peak_kpa`, `hold_leak_kpa_s`, `rise_time_90_s`, `fall_time_10_s`, `residual_kpa`, `twin_rmse_kpa`) to CSV and JSON:

```bash
python -m digital_twin.endurance_test --port SIM --cycles 50
# With real hardware (e.g., 500 cycles at 80 kPa closed-loop target + raw 50 Hz log):
python -m digital_twin.endurance_test --port COM7 --cycles 500 --mode target --target-kpa 80 --max-kpa 150 --raw-log
```

## Project Structure

- `include/msp/pressure_sensor_spi.h` — ABP2 `T150PGSA3XX` SPI non-blocking state machine & kPa scaling
- `src/tests/main_test_stm32u5xx_valve_single.c` — STM32U545 valve PWM, closed-loop pressure target, overpressure safety, and 50 Hz UART DMA telemetry
- `digital_twin/protocol.py` — ASCII protocol parser & command builders
- `digital_twin/pneumatics.py` — Lumped-parameter RC pneumatic chamber model & soft-finger pressure-to-flexion kinematics
- `digital_twin/serial_bridge.py` — Threaded `pyserial` / Win32 `ctypes` / `SIM` hardware bridge
- `digital_twin/glove_model.py` & `digital_twin/assets/robotic_glove.urdf` — 5-finger articulated robotic glove in PyBullet
- `digital_twin/sync_engine.py` — Bidirectional synchronization engine, RMSE/MAE/jitter/RTT/correlation metrics, and CSV logger
- `digital_twin/app.py` — Real-time Tkinter + PyBullet Digital Twin GUI
- `digital_twin/evaluate_sync.py` — Headless synchronization benchmark CLI
- `tests/test_digital_twin.py` — Unit & integration test suite (`python -m unittest discover -s tests -v`)
- `standalone_build/` — Standalone Windows/ARM build scripts & ST HAL/CMSIS compatibility layer producing `devel/test_stm32u5xx_valve_single.hex`

## Building & Flashing the STM32U545 Firmware

### Option A: Rebuild Firmware Locally (Standalone ARM GCC)

```powershell
python standalone_build/build_firmware.py
```

Outputs are placed in `devel/`:
- `devel/test_stm32u5xx_valve_single.hex`
- `devel/test_stm32u5xx_valve_single.bin`
- `devel/test_stm32u5xx_valve_single.elf`

### Option B: Flash with STM32CubeProgrammer

1. Connect the STM32U545 Nucleo board to your PC via USB (ST-LINK).
2. Open **STM32CubeProgrammer**, select **ST-LINK**, and click **Connect**.
3. Open **Erasing & Programming** (left sidebar download icon), browse to `devel/test_stm32u5xx_valve_single.hex`, check **Verify programming** and **Run after programming**, and click **Start Programming**.
4. Note the ST-LINK Virtual COM Port in Windows Device Manager (e.g., `COM3`) and start `python -m digital_twin.app --port COM3`.
