"""Standalone ARM build script for STM32U545xx valve & ABP2 pressure firmware."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SB = ROOT / "standalone_build"
DEPS = SB / "deps"
HAL_DIR = DEPS / "stm32u5xx-hal-driver"
CMSIS_DEV = DEPS / "cmsis-device-u5"
CMSIS_CORE = DEPS / "cmsis-core"
OUT_OBJ = SB / "obj"
DEVEL = ROOT / "devel"


def find_arm_toolchain() -> Path:
    local_dir = DEPS / "arm-gcc"
    if local_dir.exists():
        for match in sorted(local_dir.rglob("arm-none-eabi-gcc.exe"), reverse=True):
            return match.parent

    exe = shutil.which("arm-none-eabi-gcc")
    if exe:
        return Path(exe).parent

    candidates = [
        Path(r"C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi"),
        Path(r"C:\Program Files\Arm GNU Toolchain arm-none-eabi"),
    ]
    for base in candidates:
        if base.exists():
            for match in sorted(base.rglob("arm-none-eabi-gcc.exe"), reverse=True):
                return match.parent

    raise FileNotFoundError(
        "arm-none-eabi-gcc not found. Install Arm GNU Embedded Toolchain or add it to PATH."
    )


def main() -> int:
    toolchain_bin = find_arm_toolchain()
    gcc = str(toolchain_bin / "arm-none-eabi-gcc.exe")
    objcopy = str(toolchain_bin / "arm-none-eabi-objcopy.exe")
    size_tool = str(toolchain_bin / "arm-none-eabi-size.exe")

    OUT_OBJ.mkdir(parents=True, exist_ok=True)
    DEVEL.mkdir(parents=True, exist_ok=True)

    mcu_flags = [
        "-mcpu=cortex-m33",
        "-mthumb",
        "-mfpu=fpv5-sp-d16",
        "-mfloat-abi=hard",
    ]
    defines = [
        "-DSTM32U545xx",
        "-DUSE_HAL_DRIVER",
    ]
    includes = [
        f"-I{ROOT / 'include'}",
        f"-I{SB / 'compat'}",
        f"-I{HAL_DIR / 'Inc'}",
        f"-I{HAL_DIR / 'Inc' / 'Legacy'}",
        f"-I{CMSIS_DEV / 'Include'}",
        f"-I{CMSIS_CORE / 'CMSIS' / 'Core' / 'Include'}",
    ]
    cflags = [
        *mcu_flags,
        *defines,
        *includes,
        "-Os",
        "-g3",
        "-ffunction-sections",
        "-fdata-sections",
        "-Wall",
        "-Wextra",
        "-std=gnu11",
    ]

    hal_sources = [
        "stm32u5xx_hal.c",
        "stm32u5xx_hal_cortex.c",
        "stm32u5xx_hal_dma.c",
        "stm32u5xx_hal_dma_ex.c",
        "stm32u5xx_hal_exti.c",
        "stm32u5xx_hal_flash.c",
        "stm32u5xx_hal_flash_ex.c",
        "stm32u5xx_hal_gpio.c",
        "stm32u5xx_hal_icache.c",
        "stm32u5xx_hal_pwr.c",
        "stm32u5xx_hal_pwr_ex.c",
        "stm32u5xx_hal_rcc.c",
        "stm32u5xx_hal_rcc_ex.c",
        "stm32u5xx_hal_spi.c",
        "stm32u5xx_hal_spi_ex.c",
        "stm32u5xx_hal_tim.c",
        "stm32u5xx_hal_tim_ex.c",
        "stm32u5xx_hal_uart.c",
        "stm32u5xx_hal_uart_ex.c",
        "stm32u5xx_hal_usart.c",
        "stm32u5xx_hal_usart_ex.c",
    ]

    c_sources: list[Path] = [
        ROOT / "src" / "tests" / "main_test_stm32u5xx_valve_single.c",
        CMSIS_DEV / "Source" / "Templates" / "system_stm32u5xx.c",
        SB / "syscalls.c",
        *[HAL_DIR / "Src" / name for name in hal_sources],
    ]
    asm_sources: list[Path] = [
        CMSIS_DEV / "Source" / "Templates" / "gcc" / "startup_stm32u545xx.s",
    ]

    objects: list[Path] = []
    for src in c_sources:
        obj = OUT_OBJ / f"{src.stem}.o"
        objects.append(obj)
        cmd = [gcc, *cflags, "-c", str(src), "-o", str(obj)]
        res = subprocess.run(cmd, check=False)
        if res.returncode != 0:
            return res.returncode

    for asm in asm_sources:
        obj = OUT_OBJ / f"{asm.stem}.o"
        objects.append(obj)
        cmd = [gcc, *mcu_flags, "-x", "assembler-with-cpp", "-c", str(asm), "-o", str(obj)]
        res = subprocess.run(cmd, check=False)
        if res.returncode != 0:
            return res.returncode

    ld_script = CMSIS_DEV / "Source" / "Templates" / "gcc" / "linker" / "STM32U545xx_FLASH.ld"
    elf_path = DEVEL / "test_stm32u5xx_valve_single.elf"
    hex_path = DEVEL / "test_stm32u5xx_valve_single.hex"
    bin_path = DEVEL / "test_stm32u5xx_valve_single.bin"
    map_path = DEVEL / "test_stm32u5xx_valve_single.map"

    link_cmd = [
        gcc,
        *mcu_flags,
        *[str(o) for o in objects],
        f"-T{ld_script}",
        "-specs=nano.specs",
        "-u",
        "_printf_float",
        "-u",
        "_scanf_float",
        "-Wl,--gc-sections",
        f"-Wl,-Map={map_path}",
        "-Wl,--print-memory-usage",
        "-lm",
        "-o",
        str(elf_path),
    ]
    res = subprocess.run(link_cmd, check=False)
    if res.returncode != 0:
        return res.returncode

    subprocess.run([objcopy, "-O", "ihex", str(elf_path), str(hex_path)], check=True)
    subprocess.run([objcopy, "-O", "binary", str(elf_path), str(bin_path)], check=True)
    subprocess.run([size_tool, str(elf_path)], check=True)

    print(f"\nBuilt firmware:\n  HEX: {hex_path}\n  BIN: {bin_path}\n  ELF: {elf_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
