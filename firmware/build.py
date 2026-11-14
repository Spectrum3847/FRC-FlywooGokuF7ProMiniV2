#!/usr/bin/env python3
"""Build GokuIMU firmware. Usage: python build.py [flash] [--port COMx]

Toolchain/deps are expected in ../tools (see README): Arm GNU toolchain,
tinyusb, cmsis_device_f7, CMSIS_6, dfu-util.
"""
import glob
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.normpath(os.path.join(HERE, "..", "tools"))
BUILD = os.path.join(HERE, "build")
EXE = ".exe" if os.name == "nt" else ""


def tool(name):
    local = os.path.join(TOOLS, "bin", "arm-none-eabi-" + name + EXE)
    return local if os.path.exists(local) else "arm-none-eabi-" + name


TUSB = os.path.join(TOOLS, "tinyusb", "src")
SOURCES = [
    *glob.glob(os.path.join(HERE, "src", "*.c")),
    os.path.join(TUSB, "tusb.c"),
    os.path.join(TUSB, "common", "tusb_fifo.c"),
    os.path.join(TUSB, "device", "usbd.c"),
    os.path.join(TUSB, "device", "usbd_control.c"),
    os.path.join(TUSB, "class", "cdc", "cdc_device.c"),
    os.path.join(TUSB, "portable", "synopsys", "dwc2", "dcd_dwc2.c"),
    os.path.join(TUSB, "portable", "synopsys", "dwc2", "dwc2_common.c"),
    os.path.join(TOOLS, "cmsis_device_f7", "Source", "Templates", "gcc", "startup_stm32f722xx.s"),
]
INCLUDES = [
    os.path.join(HERE, "src"),
    TUSB,
    os.path.join(TOOLS, "cmsis_device_f7", "Include"),
    os.path.join(TOOLS, "CMSIS_6", "CMSIS", "Core", "Include"),
]
CPU = ["-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-sp-d16", "-mfloat-abi=hard"]
CFLAGS = CPU + [
    "-O2", "-g3", "-std=gnu11", "-Wall", "-Wextra", "-Wno-unused-parameter",
    "-ffunction-sections", "-fdata-sections",
    "-DSTM32F722xx",
] + ["-I" + i for i in INCLUDES]
LDFLAGS = CPU + [
    "-T" + os.path.join(HERE, "stm32f722.ld"), "-Wl,--gc-sections",
    "-Wl,-Map=" + os.path.join(BUILD, "gokuimu.map"), "--specs=nano.specs",
    "--specs=nosys.specs", "-Wl,--print-memory-usage",
]


def run(cmd):
    r = subprocess.run(cmd)
    if r.returncode != 0:
        sys.exit(r.returncode)


def build():
    os.makedirs(BUILD, exist_ok=True)
    objs = []
    for src in SOURCES:
        obj = os.path.join(BUILD, os.path.basename(src) + ".o")
        objs.append(obj)
        if os.path.exists(obj) and os.path.getmtime(obj) > max(
            [os.path.getmtime(src)] + [os.path.getmtime(h) for h in glob.glob(os.path.join(HERE, "src", "*.h"))]
        ):
            continue
        print("CC", os.path.relpath(src, HERE))
        run([tool("gcc")] + CFLAGS + ["-c", src, "-o", obj])
    elf = os.path.join(BUILD, "gokuimu.elf")
    run([tool("gcc")] + objs + LDFLAGS + ["-o", elf, "-lm"])
    run([tool("objcopy"), "-O", "binary", elf, os.path.join(BUILD, "gokuimu.bin")])
    run([tool("objcopy"), "-O", "ihex", elf, os.path.join(BUILD, "gokuimu.hex")])
    print("OK ->", os.path.join(BUILD, "gokuimu.bin"))


def flash():
    dfu = os.path.join(TOOLS, "dfu", "dfu-util-0.11-binaries", "win64", "dfu-util.exe") if os.name == "nt" else "dfu-util"
    run([dfu, "-a", "0", "-d", "0483:df11", "-s", "0x08000000:leave", "-D",
         os.path.join(BUILD, "gokuimu.bin")])


if __name__ == "__main__":
    build()
    if "flash" in sys.argv:
        flash()
