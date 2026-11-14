# GokuIMU: a cheap USB IMU for FRC (Flywoo Goku F7 Pro Mini V2)

This project replaces the Betaflight firmware on a **Flywoo Goku F7 Pro Mini V2** flight
controller so the board works as a USB IMU for a **Systemcore** robot controller (or a Raspberry
Pi 5 running the Systemcore image). The **vendordep** gives robot code a Pigeon 2-style API.

It is meant for training robots, where a Pigeon 2 costs more than it is worth.

| | |
|---|---|
| MCU | STM32F722RET6, 168 MHz |
| IMU | TDK ICM-42688-P, sampled at 1 kHz, ±2000 dps, ±16 g |
| Link | USB CDC (shows up as `/dev/ttyACM*`), 1–1000 Hz stream, 100 Hz by default |
| Fusion | 6-axis complementary filter on the board: gravity corrects pitch/roll, yaw comes from the gyro only |
| Calibration | Gyro bias is recalibrated automatically whenever the board is still (Pigeon "no-motion" calibration) |
| Versions | firmware 0.2.0, vendordep 0.2.2 (protocol v2: both must be 0.2+), WPILib 2027.0.0-alpha-8, Java only |

## Layout

```
firmware/            bare-metal C firmware (TinyUSB + CMSIS, no HAL)
  src/protocol.h     wire protocol (shared contract with the vendordep)
  build.py           build + flash script
  host/gokuimu.py    desktop test tool (pyserial): monitor, bench, config, dfu...
vendordep/
  src/main/java/io/github/frcgokuimu/   GokuIMU, GokuIMUConfiguration, ImuSignal, GokuIMUSimState
  build.py           builds the jar into vendordep/repo (a Maven repo) and writes GokuIMU.json
backup/              Betaflight `diff all` taken from the dev board before reflashing
tools/               (gitignored) Arm GCC 14.2, TinyUSB 0.18, CMSIS, dfu-util 0.11
```

## Flashing a board

1. Get the tools into `tools/`: the Arm GNU Toolchain 14.2 `mingw-w64-x86_64-arm-none-eabi` zip
   unzipped into `tools/`, TinyUSB 0.18.0 cloned into `tools/tinyusb`, `cmsis_device_f7`, the
   `CMSIS/Core` folder of CMSIS_6, and dfu-util 0.11 in `tools/dfu`.
2. Put the board in DFU mode:
   * **Betaflight board:** type `bl` in the Betaflight CLI, or hold the BOOT button while
     plugging in USB.
   * **Board already on GokuIMU:** run `python firmware/host/gokuimu.py dfu`, or call
     `imu.rebootToBootloader()` from robot code.
3. `cd firmware && python build.py flash`

Windows needs the STM32 DFU (WinUSB) driver. Betaflight Configurator or ImpulseRC Driver Fixer
installs it. To go back to Betaflight, flash it with Betaflight Configurator as usual.

**LED:** fast blink = still calibrating the gyro bias (keep it still for about 1 s). Slow blink =
running. Solid = IMU hardware fault.

## Mounting

The axes are **X = the arrow on the board (forward), Y = left, Z = up**, with counter-clockwise
positive, the same as a Pigeon 2 and WPILib. If the board is mounted differently, set
`MountPose`. For example, if the arrow points to the robot's left, set `MountPoseYaw = 90`.

## Using the vendordep

**Install:** in VS Code, run *WPILib: Manage Vendor Libraries* → *Install new library (online)*,
and paste this URL:

```
https://spectrum3847.github.io/FRC-FlywooGokuF7ProMiniV2/vendordep/GokuIMU.json
```

You can also download that file into your robot project's `vendordeps/` folder. GradleRIO fetches
the jar from the Maven repo that this repository serves on GitHub Pages (`vendordep/repo/`).

**Publishing a new version:**
1. Bump `VERSION` in `vendordep/build.py`.
2. Run `python vendordep/build.py`, which rebuilds the jar into `vendordep/repo/` and regenerates
   `GokuIMU.json`.
3. Commit and push. GitHub Pages serves the new files within a minute or two.
4. Leave older version folders in `vendordep/repo/`, so robot projects pinned to them still
   build.

**Firmware binaries** are attached to each GitHub release.

**Flashing a prebuilt release:**
1. Put the board in DFU mode.
2. Run `dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D gokuimu-<version>.bin`, or use STM32CubeProgrammer.

```java
import io.github.frcgokuimu.GokuIMU;
import io.github.frcgokuimu.GokuIMUConfiguration;

GokuIMU imu = new GokuIMU();            // first IMU found; or new GokuIMU("<serial suffix>")

// once at startup (writes flash only if the values changed)
var cfg = new GokuIMUConfiguration();
cfg.MountPose.MountPoseYaw = 0;
imu.getConfigurator().apply(cfg);

Rotation2d heading = imu.getRotation2d();        // continuous yaw, CCW+
double yaw = imu.getYaw().getValueAsDouble();    // degrees, does not wrap
double omega = imu.getAngularVelocityZWorld().getValueAsDouble();
imu.setYaw(0);                                   // BLOCKS up to 0.25 s (normally a few ms)
poseEstimator.updateWithTime(imu.getYaw().getTimestamp(), ...);  // timestamps are in Timer.getTimestamp() time
GokuIMU.Inputs in = imu.getInputs();             // every value from one frame (logging / replay)
```

### What blocks

Getters never block. They return the newest frame, which a background thread receives, and they
are thread-safe. These getters include every signal, `getRotation2d`, `getInputs`, `getAngle`,
`getRate`, `isConnected`, `isCalibrating`, `getDeviceInfo`, `getFirmwareVersion` and
`getSerialNumber`.

Methods that wait for the IMU to reply **block the calling thread**. Each one's Javadoc says so.

| Method | Blocks up to |
|---|---|
| `setYaw`, `reset`, `resetYaw`, `clearStickyFaults`, `calibrateGyro`, `reboot`, `rebootToBootloader` | 0.25 s (`DEFAULT_COMMAND_TIMEOUT`); timeout overloads exist |
| `refreshConfig` | 0.25 s, plus the time of another thread's `refreshConfig` already running on the same IMU |
| `applyConfig` / `getConfigurator().apply`, `factoryReset` | 3 s (`DEFAULT_CONFIG_TIMEOUT`), because they write flash |
| `waitForConnection(t)`, `waitForCalibration(t)`, `signal.waitForUpdate(t)` | `t` |
| `listDevices()` | instant on Linux; on Windows it runs `reg query` twice, normally about 100 ms and never more than about 6 s |
| constructor, `close()`, `setUpdateFrequency`, `updateAlerts` | no I/O on the calling thread; at most brief native calls or internal locks |

Commands from different threads don't wait for each other, and all USB writes happen on a
background thread, so each timeout is a true maximum. That holds even while the IMU is busy
saving a config. Replies normally arrive in a few milliseconds, so the timeouts only come into
play when the IMU is unplugged or busy.

In simulation, the getters and the `GokuIMUSimState` methods share a short internal lock. The
`waitFor*` helpers sleep in 5 ms steps, and they only finish early if another thread or the sim
GUI changes the simulated state. Call `applyConfig` and the `waitFor*` helpers at startup, not in periodic code.

```java
// Robot constructor: optional startup wait
imu.waitForConnection(2.0);      // blocks up to 2 s
imu.waitForCalibration(3.0);     // blocks up to 3 s; robot must be still
```

### Simulation

In desktop simulation, `new GokuIMU()` uses a simulated IMU automatically.

* **Physics hookup:** `getSimState().updateHeading(heading, omegaRadPerSec)` for 2D drivetrains,
  or `update(Rotation3d pose, wx, wy, wz)` for full 3D. Both keep the raw yaw continuous. The older
  setters are also there: `setRawYaw`, `addYaw`, `setAngularVelocityZ`, and the `OnboardIMUSim`
  style `setAngleZ` / `setGyroRateZ` / `setAccelZ`.
* **Raw yaw vs `setYaw`:** robot code's `setYaw` / `resetYaw` keeps an offset on top of the raw
  yaw, as on hardware. Feeding the drivetrain heading in every loop therefore doesn't undo a
  `resetYaw()` at the start of auto.
* **Simulated faults:** `setConnected(false)` makes `isConnected()` false, makes commands return
  false, freezes the data and raises the disconnect alert. `setHardwareFault(true)` and
  `setCalibrated(false)` are also available.
* **Sim time:** accumulated angles and timestamps follow simulation time, so they behave
  correctly when the sim is paused or stepped.
* **Sim GUI:** values appear as the device `GokuIMU:default`, or `GokuIMU:<serial>` when you pass
  one.
* **Unit tests:** call `HAL.initialize()` once, and `close()` each IMU in `@AfterEach`. A second
  IMU with the same name throws a clear error.
* **Real IMU from desktop sim (bench testing):** use `new GokuIMU("", GokuIMU.Mode.HARDWARE)`, or
  run with `-Dgokuimu.mode=HARDWARE`. On Windows the board's COM port is found automatically.

`vendordep/run-desktop-test.sh [COMx]` runs the simulation tests, plus hardware tests against a
plugged-in board, using WPILib's desktop native libraries.

### Logging and dashboards

`GokuIMU` implements `TelemetryLoggable`, so `Telemetry.log("imu", imu)` and Epilogue `@Logged`
both work.

* It logs as telemetry type `Gyro` with a `Value` key (yaw in degrees), so dashboards show a gyro
  widget.
* It also logs `Rotation2d` and `Rotation3d` structs, rates, acceleration, faults and latency, all
  taken from one frame.
* `getInputs()` returns the same frame as an immutable record for AdvantageKit-style replay.
* Dashboard alerts (disconnected, hardware fault, reboot while enabled) update on their own 4
  times a second, whether or not the IMU is logged.

### Porting from Pigeon 2

| Pigeon 2 (Phoenix 6) | GokuIMU |
|---|---|
| `new Pigeon2(id, "canivore")` | `new GokuIMU()` / `new GokuIMU(serial)` / `new GokuIMU("COM6", Mode.HARDWARE)` |
| `getYaw()`, `getPitch()`, `getRoll()` → `StatusSignal<Angle>` | same names → `ImuSignal<Angle>` with `getValue()`, `getValueAsDouble()`, `getTimestamp()`, `refresh()` |
| `getQuatW/X/Y/Z`, `getGravityVectorX/Y/Z`, `getAccumGyroX/Y/Z` | same |
| `getAngularVelocity{X,Y,Z}World/Device`, `getAcceleration{X,Y,Z}` | same |
| `getTemperature`, `getNoMotionCount`, `getNoMotionEnabled`, `getUpTime` | same (`getNoMotionEnabled` = calibrated; also `getIsCalibrated`) |
| `getFault_*` / `getStickyFault_*`, `clearStickyFaults()` | Hardware, BootDuringEnable, SaturatedGyroscope/Accelerometer, BootupGyroscope, LoopTimeSlow |
| `setYaw`, `reset`, `getRotation2d`, `getRotation3d` | same, including `setYaw(deg, timeoutSec)` |
| `getConfigurator().apply(Pigeon2Configuration)` | `getConfigurator().apply(GokuIMUConfiguration)`: `MountPose`, `GyroTrim`, `Features` |
| `BaseStatusSignal.getLatencyCompensatedValue(yaw, rate)` | `ImuSignal.getLatencyCompensatedValue(yaw, rate)` |
| `signal.setUpdateFrequency(hz)` | same (it sets the whole stream rate) |
| `getSimState().setRawYaw(...)` | same (raw yaw; `setYaw` keeps its offset), plus `update`, `updateHeading` and simulated faults |
| `getVersion()` etc. | `getDeviceInfo()`: firmware, serial and board, read once on connect; non-blocking |

These Pigeon 2 features are **not** available: CAN / CANivore / Phoenix Tuner, the magnetometer and
compass, supply voltage, and the factory temperature compensation (continuous no-motion bias
tracking stands in for it).

The WPILib `OnboardIMU`-style methods are also there: `getYawRadians`, `resetYaw`,
`getQuaternion`, `getAngleX/Y/Z`, `getGyroRateX/Y/Z` (rad/s), and `getAccelX/Y/Z` (m/s²).

There are also gyro-style helpers:

* `getAngle()`: continuous yaw in degrees.
* `getRate()`: vertical yaw rate in deg/s, CCW positive. The old WPILib `Gyro` interface was CW
  positive, so negate it for legacy code.
* `rotation2dSupplier()`.
* `isCalibrating()`.
* `GokuIMU.listDevices()`: the port and serial number of each attached board, on Linux and
  Windows.

### Gyro scale calibration (recommended per board)

MEMS gyros have about 0.5% scale error, which is roughly 2° per full turn. To trim it:

1. Zero the yaw.
2. Spin the robot exactly 10 turns in either direction and read the yaw.
3. Add `(3600 - |measured|) / 10` to `GyroTrim.GyroScalarZ` and apply the config.

## Measured on the dev board (stationary, on a desk)

* Sensor rate 1000 Hz. Stream at 100 Hz and at 1 kHz.
* Yaw drift 0.0014°/min over 60 s still, after auto-calibration (0.01°/min in shorter runs).
* About 7% of frames were dropped on Windows with pyserial. This hasn't been checked on Linux yet.
* `setYaw` round trip, config save to flash, survival across reboot, and reboot-to-DFU firmware
  updates all work.
* With the Java vendordep on Windows, `setYaw` is confirmed in about 15–20 ms, and 50
  back-to-back commands were all confirmed. The stream rate set with `setUpdateFrequency` is kept
  after `applyConfig`.

## Not yet tested

* **Not yet run on a real Systemcore.** The vendordep opens `/dev/ttyACM*` directly and runs
  `stty raw -echo`, and it finds boards through `/sys/class/tty/*/device/../product`. If the
  robot program's user can't open the device, add this udev rule:
  `SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0001", MODE="0666"`
* Not yet tested on a moving robot: dynamic accuracy, tilt under acceleration, gyro trim.
* There is no C++ library yet.
* The USB VID/PID is the pid.codes test pair `1209:0001`, which is fine for private use. Devices
  are matched by the product string "GokuIMU".
* Saving a config erases a 128 KiB flash sector, which pauses the IMU for about 1 s. Apply
  configs only at startup.
