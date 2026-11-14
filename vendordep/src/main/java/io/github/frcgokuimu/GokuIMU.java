package io.github.frcgokuimu;

import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Gs;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.function.ToDoubleFunction;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Time;
import org.wpilib.util.Alert;

/**
 * A Flywoo Goku F7 Pro Mini V2 running GokuIMU firmware, connected to the robot controller over
 * USB. API mirrors CTRE's Phoenix 6 {@code Pigeon2} (signals, setYaw, configs, faults, sim state)
 * plus WPILib's {@code OnboardIMU} convenience methods.
 *
 * <p>Frame: X forward (arrow on the board), Y left, Z up; counter-clockwise positive. Yaw is
 * continuous (does not wrap at 360), like the Pigeon 2.
 *
 * <p><b>Blocking:</b> every getter (signals, getRotation2d, getInputs, isConnected, ...) is
 * non-blocking and thread-safe; it returns the newest frame received by a background thread and
 * never does I/O. (In simulation, getters take a short internal lock shared with the sim state.)
 * Methods that talk to the IMU and wait for its reply (setYaw, applyConfig, calibrateGyro, ...)
 * <b>block the calling thread</b> for up to their timeout, which is called out on each one.
 * Commands from different threads don't wait for each other, so each timeout is a true maximum.
 * Typical replies take a few milliseconds; the timeout only matters when the IMU is unplugged.
 *
 * <p><b>Threads and process-wide effects:</b> each hardware IMU runs a daemon reader thread and a
 * daemon writer thread. One shared daemon thread updates the dashboard {@code Alert}s 4 times a
 * second, so alerts change from that thread, not the robot main loop. On Linux, loading this
 * library makes the whole robot process ignore SIGHUP: Java can't open a tty without it possibly
 * becoming the controlling terminal, and unplugging the IMU would otherwise send SIGHUP and kill
 * the robot program.
 *
 * <p><b>Finding the IMU:</b> on Linux, boards are found by their USB product string "GokuIMU"
 * (from sysfs), not by VID/PID, so other USB-serial devices on /dev/ttyACM* (e.g. a CANable) are
 * never opened. On Windows, they are found by VID/PID 1209:0001 in the registry.
 *
 * <pre>{@code
 * GokuIMU imu = new GokuIMU();
 * Rotation2d heading = imu.getRotation2d();
 * double yawDeg = imu.getYaw().getValueAsDouble();
 * imu.setYaw(0); // blocks up to 0.25 s
 * }</pre>
 */
public final class GokuIMU implements AutoCloseable, TelemetryLoggable {
  /** Whether to talk to a real IMU or the simulated one. */
  public enum Mode {
    /** Real IMU on a robot, simulated IMU in desktop simulation (default). */
    AUTO,
    /** Always use a real USB IMU, even in desktop simulation (bench testing). */
    HARDWARE,
    /** Always simulate. */
    SIMULATION
  }

  /** Static information about a connected IMU, read once per connection. */
  public record DeviceInfo(
      String firmwareVersion, String serialNumber, String board, int imuWhoAmI, int protocol) {}

  /** An IMU the operating system can see: its port and USB serial number. */
  public record DevicePath(String path, String serialNumber) {}

  /**
   * Every value from a single IMU frame, so they are consistent with each other (good for logging
   * and AdvantageKit-style replay). Angles in degrees, rates in deg/s, acceleration in g.
   *
   * @param faultFlags active faults only ({@code FAULT_*} bits); 0 means healthy
   * @param stickyFaultFlags faults seen since the last clearStickyFaults() ({@code FAULT_*} bits)
   * @param statusFlags non-fault status bits ({@code STATUS_*}), e.g. {@link #STATUS_CALIBRATED}
   */
  public record Inputs(
      double timestamp,
      boolean connected,
      boolean calibrated,
      boolean moving,
      double yawDeg,
      double pitchDeg,
      double rollDeg,
      double qw,
      double qx,
      double qy,
      double qz,
      double rateXDeviceDps,
      double rateYDeviceDps,
      double rateZDeviceDps,
      double yawRateWorldDps,
      double accelXG,
      double accelYG,
      double accelZG,
      double temperatureC,
      int faultFlags,
      int stickyFaultFlags,
      int statusFlags)
      implements TelemetryLoggable {
    public Rotation2d getRotation2d() {
      return Rotation2d.fromDegrees(yawDeg);
    }

    public Rotation3d getRotation3d() {
      return new Rotation3d(new Quaternion(qw, qx, qy, qz));
    }

    @Override
    public void logTo(TelemetryTable table) {
      table.log("Timestamp", timestamp);
      table.log("Connected", connected);
      table.log("Calibrated", calibrated);
      table.log("Moving", moving);
      table.log("Yaw", yawDeg);
      table.log("Pitch", pitchDeg);
      table.log("Roll", rollDeg);
      table.log("Rotation2d", getRotation2d(), Rotation2d.struct);
      table.log("Rotation3d", getRotation3d(), Rotation3d.struct);
      table.log("RateDevice", new double[] {rateXDeviceDps, rateYDeviceDps, rateZDeviceDps});
      table.log("YawRateWorld", yawRateWorldDps);
      table.log("Accel", new double[] {accelXG, accelYG, accelZG});
      table.log("Temperature", temperatureC);
      table.log("FaultFlags", faultFlags);
      table.log("StickyFaultFlags", stickyFaultFlags);
      table.log("StatusFlags", statusFlags);
    }
  }

  // Bit values for Inputs.faultFlags / stickyFaultFlags (faults) and statusFlags (status).
  /** Fault: IMU chip not responding. */
  public static final int FAULT_HARDWARE = Protocol.FLAG_HW_FAULT;
  /** Fault: gyro hit its +/-2000 deg/s limit. */
  public static final int FAULT_SATURATED_GYRO = Protocol.FLAG_GYRO_SAT;
  /** Fault: accelerometer hit its +/-16 g limit. */
  public static final int FAULT_SATURATED_ACCEL = Protocol.FLAG_ACCEL_SAT;
  /** Fault: crystal failed, running on the internal clock. */
  public static final int FAULT_CLOCK = Protocol.FLAG_HSE_FAIL;
  /** Fault: a sensor sample was late or missed. */
  public static final int FAULT_SAMPLE_MISSED = Protocol.FLAG_SAMPLE_MISSED;
  /** Status: gyro bias calibration has completed (the normal, healthy state). */
  public static final int STATUS_CALIBRATED = Protocol.FLAG_BIAS_VALID;
  /** Status: the IMU sees motion. */
  public static final int STATUS_MOVING = Protocol.FLAG_MOVING;
  /** Status: first few seconds after the IMU booted. */
  public static final int STATUS_BOOTING = Protocol.FLAG_BOOTING;
  /** Status: no saved configuration on the IMU, defaults in use. */
  public static final int STATUS_DEFAULT_CONFIG = Protocol.FLAG_CFG_DEFAULT;

  private static final int FAULT_MASK =
      FAULT_HARDWARE | FAULT_SATURATED_GYRO | FAULT_SATURATED_ACCEL | FAULT_CLOCK
          | FAULT_SAMPLE_MISSED;

  /** Latest decoded frame plus its estimated measurement time in robot time. */
  record Snapshot(Protocol.State s, double robotTime) {}

  /** Default timeout for commands like setYaw, seconds. */
  public static final double DEFAULT_COMMAND_TIMEOUT = 0.25;
  /** Default timeout for applying a configuration (includes a flash write), seconds. */
  public static final double DEFAULT_CONFIG_TIMEOUT = 3.0;

  private static final double MIN_STALE_SECONDS = 0.25;
  private static final AtomicInteger INSTANCES = new AtomicInteger();
  private static final ScheduledExecutorService ALERT_EXECUTOR =
      Executors.newSingleThreadScheduledExecutor(
          r -> {
            Thread t = new Thread(r, "GokuIMU-alerts");
            t.setDaemon(true);
            return t;
          });

  private final String m_selector;
  private final SerialLink m_link;
  private final GokuIMUSimState m_sim;
  private volatile Snapshot m_snapshot = new Snapshot(Protocol.ZERO, 0);
  private volatile double m_lastRxTime = -1;
  private final Object m_updateLock = new Object();

  // Clock sync (reader thread only): windowed minimum of (rxTime - deviceTime).
  private long m_devTimeHigh;
  private long m_lastDevUs = -1;
  private double m_offset = Double.NaN;
  private double m_windowMin = Double.POSITIVE_INFINITY;
  private double m_windowStart;

  private volatile boolean m_bootDuringEnable;
  private volatile double m_bootDetectedTime = Double.NEGATIVE_INFINITY;
  private long m_lastUptime = -1;
  private volatile boolean m_enabledAtLastFrame;
  private volatile double m_frameInterval = 0.01; // smoothed seconds between frames
  private volatile double m_requestedRateHz = -1;
  private volatile boolean m_ratePending; // a rate change couldn't be queued; retried at 4 Hz
  private volatile DeviceInfo m_info;

  // Request/reply matching. Every command carries a sequence byte the IMU echoes in its ACK.
  private final Object m_configLock = new Object(); // config replies carry no sequence byte
  private final Object m_alertLock = new Object();
  private volatile boolean m_closed;
  private final AtomicInteger m_seq = new AtomicInteger();
  private final Map<Integer, CompletableFuture<byte[]>> m_pendingAcks = new ConcurrentHashMap<>();
  private volatile CompletableFuture<byte[]> m_configFuture;

  private final Alert m_disconnectedAlert;
  private final Alert m_faultAlert;
  private final Alert m_bootAlert;
  private final ScheduledFuture<?> m_alertTask;

  /**
   * Uses the first GokuIMU found ({@link Mode#AUTO}). Constructors return quickly: they do no I/O
   * on the calling thread (finding and opening the port happens on a background thread), only
   * brief native setup for alerts and, in simulation, the sim device.
   */
  public GokuIMU() {
    this("", Mode.AUTO);
  }

  /**
   * @param serialOrPort the board's USB serial number (or its last few characters, see {@link
   *     #listDevices()}), or a port like "/dev/ttyACM0" or "COM6". Empty for the first IMU found.
   */
  public GokuIMU(String serialOrPort) {
    this(serialOrPort, Mode.AUTO);
  }

  /**
   * @param serialOrPort see {@link #GokuIMU(String)}
   * @param mode real or simulated IMU. The system property {@code gokuimu.mode}
   *     (AUTO/HARDWARE/SIMULATION) overrides {@link Mode#AUTO}, e.g. {@code
   *     -Dgokuimu.mode=HARDWARE} to bench-test a real IMU from desktop simulation.
   */
  public GokuIMU(String serialOrPort, Mode mode) {
    m_selector = serialOrPort == null ? "" : serialOrPort;
    if (mode == Mode.AUTO) {
      String prop = System.getProperty("gokuimu.mode", "AUTO");
      try {
        mode = Mode.valueOf(prop.trim().toUpperCase(java.util.Locale.ROOT));
      } catch (IllegalArgumentException e) {
        System.err.println(
            "GokuIMU: ignoring invalid gokuimu.mode=" + prop + " (use AUTO, HARDWARE or SIMULATION)");
        mode = Mode.AUTO;
      }
    }
    boolean simulate = mode == Mode.SIMULATION || (mode == Mode.AUTO && RobotBase.isSimulation());

    String label = m_selector.isEmpty() ? "" : " (" + m_selector + ")";
    // Alert ids: no '/' (from /dev/ttyACM0) so they make clean NetworkTables paths.
    String id =
        "GokuIMU_" + m_selector.replaceAll("[^A-Za-z0-9]", "_") + "_" + INSTANCES.incrementAndGet();
    m_disconnectedAlert =
        new Alert(id + ".disconnected", "GokuIMU" + label + " disconnected", Alert.Level.HIGH);
    m_faultAlert =
        new Alert(id + ".fault", "GokuIMU" + label + " hardware fault", Alert.Level.HIGH);
    m_bootAlert =
        new Alert(id + ".boot", "GokuIMU" + label + " rebooted while enabled", Alert.Level.MEDIUM);

    if (simulate) {
      m_link = null;
      try {
        m_sim = new GokuIMUSimState(m_selector);
      } catch (RuntimeException e) {
        m_disconnectedAlert.close(); // don't leak the alerts if the sim device can't be made
        m_faultAlert.close();
        m_bootAlert.close();
        throw e;
      }
    } else {
      m_sim = null;
      m_link = new SerialLink(m_selector, this::onFrame, this::onConnect);
    }
    // Alerts update on their own (4 Hz), whether or not the IMU is logged.
    m_alertTask =
        ALERT_EXECUTOR.scheduleAtFixedRate(this::updateAlertsQuietly, 250, 250, TimeUnit.MILLISECONDS);
  }

  // ---------------------------------------------------------------- internals

  Snapshot snapshot() {
    if (m_sim != null) {
      return new Snapshot(m_sim.state(), Timer.getTimestamp());
    }
    return m_snapshot;
  }

  /** Reader thread: a (re)connection was opened. Must not block. */
  private void onConnect() {
    m_offset = Double.NaN;
    m_windowMin = Double.POSITIVE_INFINITY;
    m_lastDevUs = -1;
    m_info = null;
    // Fail any request still waiting on the old connection.
    for (var f : m_pendingAcks.values()) {
      f.complete(null);
    }
    m_pendingAcks.clear();
    fireAndForget(Protocol.CMD_GET_INFO, null); // reply is cached by onFrame
    if (m_requestedRateHz > 0) {
      sendRate(m_requestedRateHz);
    }
  }

  /** Reader thread: one frame arrived. Must not block. */
  private void onFrame(int type, byte[] payload) {
    switch (type) {
      case Protocol.MSG_STATE -> {
        if (payload.length >= Protocol.STATE_LEN) {
          onState(Protocol.State.parse(payload));
        }
      }
      case Protocol.MSG_CONFIG -> {
        var f = m_configFuture;
        if (f != null) {
          f.complete(payload);
        }
      }
      case Protocol.MSG_INFO -> {
        DeviceInfo info = parseInfo(payload);
        if (info != null) {
          m_info = info;
        }
      }
      case Protocol.MSG_ACK -> {
        if (payload.length >= 3) {
          var f = m_pendingAcks.remove(payload[2] & 0xFF);
          if (f != null) {
            f.complete(payload);
          }
        }
      }
      default -> {}
    }
  }

  private static DeviceInfo parseInfo(byte[] p) {
    if (p.length < 20) {
      return null;
    }
    var b = Protocol.le(p);
    int proto = b.get() & 0xFF;
    String fw = (b.get() & 0xFF) + "." + (b.get() & 0xFF) + "." + (b.get() & 0xFF);
    String serial = String.format("%08X%08X%08X", b.getInt(), b.getInt(), b.getInt());
    int whoami = b.get() & 0xFF;
    b.position(b.position() + 3);
    byte[] board = new byte[Math.max(0, Math.min(24, b.remaining()))];
    b.get(board);
    String boardName = new String(board, java.nio.charset.StandardCharsets.US_ASCII).trim();
    int nul = boardName.indexOf('\0');
    if (nul >= 0) {
      boardName = boardName.substring(0, nul);
    }
    return new DeviceInfo(fw, serial, boardName, whoami, proto);
  }

  private void onState(Protocol.State s) {
    double rx = Timer.getTimestamp();
    // Extend the 32-bit microsecond device clock.
    if (m_lastDevUs >= 0 && s.deviceTimestampUs() < m_lastDevUs - 0x80000000L) {
      m_devTimeHigh += 1L << 32;
    }
    m_lastDevUs = s.deviceTimestampUs();
    double dev = (m_devTimeHigh + s.deviceTimestampUs()) * 1e-6;

    // Windowed minimum transport delay => device-to-robot clock offset.
    double off = rx - dev;
    if (off < m_windowMin) {
      m_windowMin = off;
    }
    if (Double.isNaN(m_offset) || off < m_offset) {
      m_offset = off;
    }
    if (rx - m_windowStart > 2.0) {
      m_offset = m_windowMin; // lets the estimate follow clock drift upward
      m_windowMin = Double.POSITIVE_INFINITY;
      m_windowStart = rx;
    }
    double measured = Math.min(dev + m_offset, rx);

    // Rebooted while enabled: enabled now, or enabled when the old link went quiet.
    boolean enabled = RobotBase.isEnabled();
    if (m_lastUptime >= 0
        && s.uptimeMs() + 1000 < m_lastUptime
        && (enabled || m_enabledAtLastFrame)) {
      m_bootDuringEnable = true;
      m_bootDetectedTime = rx;
    }
    m_lastUptime = s.uptimeMs();
    m_enabledAtLastFrame = enabled;
    double prevRx = m_lastRxTime;
    if (prevRx >= 0 && rx - prevRx < 2.0) {
      m_frameInterval += 0.1 * ((rx - prevRx) - m_frameInterval);
    }

    m_snapshot = new Snapshot(s, measured);
    m_lastRxTime = rx;
    synchronized (m_updateLock) {
      m_updateLock.notifyAll();
    }
  }

  /**
   * Blocks until a new frame arrives or the timeout passes. Returns true if one arrived. In
   * simulation there are no frames: sleeps up to 5 ms (so wait loops don't spin) and returns
   * whether the simulated IMU is connected.
   */
  boolean waitForUpdate(double timeoutSeconds) {
    if (m_sim != null) {
      sleepQuietly(Math.min(timeoutSeconds, 0.005));
      return m_sim.isConnected();
    }
    Snapshot before = m_snapshot;
    long deadline = System.nanoTime() + (long) (timeoutSeconds * 1e9);
    synchronized (m_updateLock) {
      while (m_snapshot == before) {
        long ms = (deadline - System.nanoTime()) / 1_000_000;
        if (ms <= 0) {
          return false;
        }
        try {
          m_updateLock.wait(ms);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          return false;
        }
      }
    }
    return true;
  }

  private int nextSeq() {
    return m_seq.incrementAndGet() & 0xFF;
  }

  private void fireAndForget(int cmd, byte[] args) {
    if (m_link != null) {
      m_link.sendFrame(Protocol.command(cmd, nextSeq(), args));
    }
  }

  /**
   * Sends a command and BLOCKS until its ACK (matched by sequence number) or the timeout. Returns
   * the ACK payload or null. No lock is held while waiting, so concurrent commands from different
   * threads proceed independently (up to 256 may be in flight).
   */
  private byte[] requestAck(int cmd, byte[] args, double timeoutSeconds) {
    if (m_link == null) {
      return null;
    }
    int seq = nextSeq();
    CompletableFuture<byte[]> f = new CompletableFuture<>();
    m_pendingAcks.put(seq, f);
    try {
      if (!m_link.sendFrame(Protocol.command(cmd, seq, args))) {
        return null;
      }
      byte[] ack = f.get((long) (timeoutSeconds * 1000), TimeUnit.MILLISECONDS);
      return ack != null && ack.length >= 3 && (ack[0] & 0xFF) == cmd ? ack : null;
    } catch (Exception e) {
      return null;
    } finally {
      m_pendingAcks.remove(seq, f);
    }
  }

  /** BLOCKS up to timeoutSeconds. */
  private boolean command(int cmd, byte[] args, double timeoutSeconds) {
    byte[] ack = requestAck(cmd, args, timeoutSeconds);
    return ack != null && ack[1] == Protocol.ACK_OK;
  }

  private void sendRate(double hz) {
    int v = (int) Math.max(1, Math.min(1000, Math.round(hz)));
    boolean queued =
        m_link != null
            && m_link.sendFrame(
                Protocol.command(Protocol.CMD_SET_RATE, nextSeq(), new byte[] {(byte) v, (byte) (v >> 8)}));
    m_ratePending = !queued; // retried from the alert tick (and always re-sent on reconnect)
  }

  private boolean simConnected() {
    return m_sim != null && m_sim.isConnected();
  }

  private static double[] worldRates(Protocol.State s) {
    // v_world = q * v_body * q^-1
    double w = s.qw(), x = s.qx(), y = s.qy(), z = s.qz();
    double vx = s.gyroX(), vy = s.gyroY(), vz = s.gyroZ();
    double tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
    return new double[] {
      vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx)
    };
  }

  private static double[] gravity(Protocol.State s) {
    double w = s.qw(), x = s.qx(), y = s.qy(), z = s.qz();
    return new double[] {2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z};
  }

  private ImuSignal<Angle> angle(String name, ToDoubleFunction<Snapshot> f) {
    return new ImuSignal<>(this, name, "deg", f, Degrees::of);
  }

  private ImuSignal<AngularVelocity> rate(String name, ToDoubleFunction<Snapshot> f) {
    return new ImuSignal<>(this, name, "deg/s", f, DegreesPerSecond::of);
  }

  private ImuSignal<Double> scalar(String name, ToDoubleFunction<Snapshot> f) {
    return new ImuSignal<>(this, name, "", f, v -> v);
  }

  private ImuSignal<Boolean> flag(String name, Predicate<Snapshot> f) {
    return new ImuSignal<>(this, name, "", s -> f.test(s) ? 1 : 0, v -> v != 0);
  }

  // ---------------------------------------------------------------- Pigeon 2 style signals
  // All signal getters are non-blocking.

  /** Continuous yaw (does not wrap), degrees, CCW positive. */
  public ImuSignal<Angle> getYaw() {
    return angle("Yaw", s -> s.s().yaw());
  }

  /** Pitch, degrees (rotation about +Y / left axis; nose down is positive). */
  public ImuSignal<Angle> getPitch() {
    return angle("Pitch", s -> s.s().pitch());
  }

  /** Roll, degrees (rotation about +X / forward axis). */
  public ImuSignal<Angle> getRoll() {
    return angle("Roll", s -> s.s().roll());
  }

  public ImuSignal<Double> getQuatW() {
    return scalar("QuatW", s -> s.s().qw());
  }

  public ImuSignal<Double> getQuatX() {
    return scalar("QuatX", s -> s.s().qx());
  }

  public ImuSignal<Double> getQuatY() {
    return scalar("QuatY", s -> s.s().qy());
  }

  public ImuSignal<Double> getQuatZ() {
    return scalar("QuatZ", s -> s.s().qz());
  }

  /** Gravity ("up") direction in the robot frame, X component (unit vector). */
  public ImuSignal<Double> getGravityVectorX() {
    return scalar("GravityVectorX", s -> gravity(s.s())[0]);
  }

  public ImuSignal<Double> getGravityVectorY() {
    return scalar("GravityVectorY", s -> gravity(s.s())[1]);
  }

  public ImuSignal<Double> getGravityVectorZ() {
    return scalar("GravityVectorZ", s -> gravity(s.s())[2]);
  }

  /** Integrated robot-frame X rate since boot, degrees. */
  public ImuSignal<Angle> getAccumGyroX() {
    return angle("AccumGyroX", s -> s.s().accumX());
  }

  public ImuSignal<Angle> getAccumGyroY() {
    return angle("AccumGyroY", s -> s.s().accumY());
  }

  public ImuSignal<Angle> getAccumGyroZ() {
    return angle("AccumGyroZ", s -> s.s().accumZ());
  }

  /** Angular velocity about the world X axis, deg/s. */
  public ImuSignal<AngularVelocity> getAngularVelocityXWorld() {
    return rate("AngularVelocityXWorld", s -> worldRates(s.s())[0]);
  }

  public ImuSignal<AngularVelocity> getAngularVelocityYWorld() {
    return rate("AngularVelocityYWorld", s -> worldRates(s.s())[1]);
  }

  /** Angular velocity about the world Z (vertical) axis, deg/s. Use with getYaw(). */
  public ImuSignal<AngularVelocity> getAngularVelocityZWorld() {
    return rate("AngularVelocityZWorld", s -> worldRates(s.s())[2]);
  }

  /** Angular velocity about the robot X axis, deg/s. */
  public ImuSignal<AngularVelocity> getAngularVelocityXDevice() {
    return rate("AngularVelocityXDevice", s -> s.s().gyroX());
  }

  public ImuSignal<AngularVelocity> getAngularVelocityYDevice() {
    return rate("AngularVelocityYDevice", s -> s.s().gyroY());
  }

  public ImuSignal<AngularVelocity> getAngularVelocityZDevice() {
    return rate("AngularVelocityZDevice", s -> s.s().gyroZ());
  }

  /** Acceleration along robot X including gravity, in g. */
  public ImuSignal<LinearAcceleration> getAccelerationX() {
    return new ImuSignal<>(this, "AccelerationX", "g", s -> s.s().accelX(), Gs::of);
  }

  public ImuSignal<LinearAcceleration> getAccelerationY() {
    return new ImuSignal<>(this, "AccelerationY", "g", s -> s.s().accelY(), Gs::of);
  }

  public ImuSignal<LinearAcceleration> getAccelerationZ() {
    return new ImuSignal<>(this, "AccelerationZ", "g", s -> s.s().accelZ(), Gs::of);
  }

  /** IMU die temperature, Celsius. */
  public ImuSignal<Temperature> getTemperature() {
    return new ImuSignal<>(this, "Temperature", "degC", s -> s.s().temperature(), Celsius::of);
  }

  /** Number of automatic no-motion gyro bias calibrations since boot. */
  public ImuSignal<Double> getNoMotionCount() {
    return scalar("NoMotionCount", s -> s.s().noMotionCount());
  }

  /**
   * True once a no-motion gyro bias calibration has completed since boot. (On a Pigeon 2 this
   * reports whether the feature is enabled; here it reports whether it has succeeded, which is
   * the more useful thing to check before a match.)
   */
  public ImuSignal<Boolean> getNoMotionEnabled() {
    return getIsCalibrated();
  }

  /** True once a gyro bias calibration has completed since the IMU booted. */
  public ImuSignal<Boolean> getIsCalibrated() {
    return flag("IsCalibrated", s -> s.s().has(Protocol.FLAG_BIAS_VALID));
  }

  /** True if the IMU currently sees motion. */
  public ImuSignal<Boolean> getIsMoving() {
    return flag("IsMoving", s -> s.s().has(Protocol.FLAG_MOVING));
  }

  /** IMU uptime. */
  public ImuSignal<Time> getUpTime() {
    return new ImuSignal<>(this, "UpTime", "s", s -> s.s().uptimeMs() / 1000.0, Seconds::of);
  }

  /** Measured sensor sample rate in Hz (nominally 1000). */
  public ImuSignal<Double> getSampleRate() {
    return scalar("SampleRate", s -> s.s().sampleRateHz());
  }

  // ---- faults (live and sticky), named like Pigeon 2. Non-blocking. ----

  public ImuSignal<Boolean> getFault_Hardware() {
    return flag("Fault_Hardware", s -> !isConnected() || s.s().has(Protocol.FLAG_HW_FAULT));
  }

  /** Live fault: the IMU rebooted while the robot was enabled, within the last 5 seconds. */
  public ImuSignal<Boolean> getFault_BootDuringEnable() {
    return flag("Fault_BootDuringEnable", s -> Timer.getTimestamp() - m_bootDetectedTime < 5.0);
  }

  public ImuSignal<Boolean> getFault_SaturatedGyroscope() {
    return flag("Fault_SaturatedGyroscope", s -> s.s().has(Protocol.FLAG_GYRO_SAT));
  }

  public ImuSignal<Boolean> getFault_SaturatedAccelerometer() {
    return flag("Fault_SaturatedAccelerometer", s -> s.s().has(Protocol.FLAG_ACCEL_SAT));
  }

  public ImuSignal<Boolean> getFault_BootupGyroscope() {
    return flag("Fault_BootupGyroscope", s -> s.s().has(Protocol.FLAG_BOOTING));
  }

  public ImuSignal<Boolean> getFault_LoopTimeSlow() {
    return flag("Fault_LoopTimeSlow", s -> s.s().has(Protocol.FLAG_SAMPLE_MISSED));
  }

  public ImuSignal<Boolean> getFault_ClockFault() {
    return flag("Fault_ClockFault", s -> s.s().has(Protocol.FLAG_HSE_FAIL));
  }

  public ImuSignal<Boolean> getStickyFault_Hardware() {
    return flag("StickyFault_Hardware", s -> s.s().sticky(Protocol.FLAG_HW_FAULT));
  }

  public ImuSignal<Boolean> getStickyFault_BootDuringEnable() {
    return flag("StickyFault_BootDuringEnable", s -> m_bootDuringEnable);
  }

  public ImuSignal<Boolean> getStickyFault_SaturatedGyroscope() {
    return flag("StickyFault_SaturatedGyroscope", s -> s.s().sticky(Protocol.FLAG_GYRO_SAT));
  }

  public ImuSignal<Boolean> getStickyFault_SaturatedAccelerometer() {
    return flag("StickyFault_SaturatedAccelerometer", s -> s.s().sticky(Protocol.FLAG_ACCEL_SAT));
  }

  public ImuSignal<Boolean> getStickyFault_LoopTimeSlow() {
    return flag("StickyFault_LoopTimeSlow", s -> s.s().sticky(Protocol.FLAG_SAMPLE_MISSED));
  }

  /**
   * Clears sticky faults on the device and the host-side boot-during-enable flag.
   *
   * <p><b>Blocks</b> the calling thread up to {@link #DEFAULT_COMMAND_TIMEOUT}.
   */
  public boolean clearStickyFaults() {
    m_bootDuringEnable = false;
    if (m_sim != null) {
      return simConnected();
    }
    return command(Protocol.CMD_CLEAR_STICKY, null, DEFAULT_COMMAND_TIMEOUT);
  }

  // ---------------------------------------------------------------- actions

  /**
   * Sets the current yaw, degrees. Also rotates the quaternion / Rotation3d to match.
   *
   * <p><b>Blocks</b> the calling thread until the IMU confirms, up to {@link
   * #DEFAULT_COMMAND_TIMEOUT} (normally a few ms).
   *
   * @return true if the IMU confirmed
   */
  public boolean setYaw(double degrees) {
    return setYaw(degrees, DEFAULT_COMMAND_TIMEOUT);
  }

  /**
   * Sets the current yaw, degrees. <b>Blocks</b> up to {@code timeoutSeconds}.
   *
   * @return true if the IMU confirmed
   */
  public boolean setYaw(double degrees, double timeoutSeconds) {
    if (!Double.isFinite(degrees) || Math.abs(degrees) > 1e7) {
      throw new IllegalArgumentException("yaw must be finite and within +/-1e7 degrees");
    }
    if (m_sim != null) {
      if (!simConnected()) {
        return false;
      }
      m_sim.setUserYaw(degrees);
      return true;
    }
    byte[] p = Protocol.le(new byte[4]).putFloat((float) degrees).array();
    return command(Protocol.CMD_SET_YAW, p, timeoutSeconds);
  }

  /** Sets the current yaw. <b>Blocks</b> up to {@link #DEFAULT_COMMAND_TIMEOUT}. */
  public boolean setYaw(Angle yaw) {
    return setYaw(yaw.in(Degrees));
  }

  /**
   * Same as setYaw(0). <b>Blocks</b> up to {@link #DEFAULT_COMMAND_TIMEOUT}.
   *
   * @return true if the IMU confirmed
   */
  public boolean reset() {
    return setYaw(0);
  }

  /**
   * Sets how often the IMU streams data, 1-1000 Hz (default 100, or the configured default). Not
   * saved across IMU reboots; re-sent automatically after a reconnect.
   *
   * <p>Non-blocking: the command is queued for a background writer thread and no reply is
   * awaited. If it can't be queued (IMU unplugged or busy), it is retried automatically.
   *
   * <p>On Windows (bench testing only), rates below ~10 Hz can delay command replies past
   * {@link #DEFAULT_COMMAND_TIMEOUT}; use the timeout overloads or a higher rate there.
   */
  public void setUpdateFrequency(double hz) {
    m_requestedRateHz = hz;
    if (m_link != null) {
      sendRate(hz);
    }
  }

  /**
   * Forces a new gyro bias calibration; the robot must then stay still for about a second (see
   * {@link #waitForCalibration}).
   *
   * <p><b>Blocks</b> up to {@link #DEFAULT_COMMAND_TIMEOUT} for the IMU to accept the command
   * (not for the calibration itself).
   */
  public boolean calibrateGyro() {
    if (m_sim != null) {
      return simConnected();
    }
    return command(Protocol.CMD_CALIBRATE_GYRO, null, DEFAULT_COMMAND_TIMEOUT);
  }

  /** Reboots the IMU. <b>Blocks</b> up to {@link #DEFAULT_COMMAND_TIMEOUT}. */
  public boolean reboot() {
    if (m_sim != null) {
      return simConnected();
    }
    return command(Protocol.CMD_REBOOT, null, DEFAULT_COMMAND_TIMEOUT);
  }

  /**
   * Reboots the IMU into the STM32 ROM DFU bootloader for firmware updates. <b>Blocks</b> up to
   * {@link #DEFAULT_COMMAND_TIMEOUT}.
   */
  public boolean rebootToBootloader() {
    if (m_sim != null) {
      return simConnected();
    }
    return command(Protocol.CMD_REBOOT_DFU, null, DEFAULT_COMMAND_TIMEOUT);
  }

  /**
   * Erases the saved configuration and returns to defaults. <b>Blocks</b> up to {@link
   * #DEFAULT_CONFIG_TIMEOUT} (includes a flash erase of about 1 s).
   */
  public boolean factoryReset() {
    if (m_sim != null) {
      if (!simConnected()) {
        return false;
      }
      m_sim.storeConfig(new GokuIMUConfiguration());
      return true;
    }
    return command(Protocol.CMD_FACTORY_RESET, null, DEFAULT_CONFIG_TIMEOUT);
  }

  // ---------------------------------------------------------------- startup helpers

  /**
   * <b>Blocks</b> the calling thread until the IMU is streaming or the timeout passes. Call once in
   * the robot constructor if you want to wait for the IMU at boot. In simulation the connection
   * only changes if another thread (or the sim GUI) calls {@link GokuIMUSimState#setConnected}, so
   * a disconnected sim IMU always waits the full timeout.
   *
   * @return true if connected
   */
  public boolean waitForConnection(double timeoutSeconds) {
    long deadline = System.nanoTime() + (long) (timeoutSeconds * 1e9);
    while (!isConnected()) {
      double left = (deadline - System.nanoTime()) * 1e-9;
      if (left <= 0) {
        return false;
      }
      waitForUpdate(Math.min(left, 0.05)); // sleeps up to 5 ms in sim, so this never spins
    }
    return true;
  }

  /** True until the first gyro bias calibration completes (keep the robot still). Non-blocking. */
  public boolean isCalibrating() {
    Protocol.State s = snapshot().s();
    return !isConnected() || !s.has(Protocol.FLAG_BIAS_VALID) || s.has(Protocol.FLAG_BOOTING);
  }

  /**
   * <b>Blocks</b> the calling thread until the gyro calibration is done (robot must be still) or
   * the timeout passes. In simulation the state only changes if another thread (or the sim GUI)
   * calls {@link GokuIMUSimState#setCalibrated}, so an uncalibrated sim IMU always waits the full
   * timeout.
   *
   * @return true if calibrated
   */
  public boolean waitForCalibration(double timeoutSeconds) {
    long deadline = System.nanoTime() + (long) (timeoutSeconds * 1e9);
    while (isCalibrating()) {
      double left = (deadline - System.nanoTime()) * 1e-9;
      if (left <= 0) {
        return false;
      }
      if (!waitForUpdate(Math.min(left, 0.05))) {
        sleepQuietly(Math.min(left, 0.02));
      }
    }
    return true;
  }

  private static void sleepQuietly(double seconds) {
    try {
      Thread.sleep(Math.max(1, (long) (seconds * 1000)));
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  // ---------------------------------------------------------------- configuration

  /**
   * Applies and saves a configuration. Only rewrites flash if something changed.
   *
   * <p><b>Blocks</b> the calling thread up to {@link #DEFAULT_CONFIG_TIMEOUT} (a flash write
   * takes about 1 s). Call once at startup, not in periodic code.
   *
   * @return true if the IMU confirmed the configuration was saved
   */
  public boolean applyConfig(GokuIMUConfiguration config) {
    return applyConfig(config, DEFAULT_CONFIG_TIMEOUT);
  }

  /** Same as {@link #applyConfig(GokuIMUConfiguration)}; <b>blocks</b> up to the timeout. */
  public boolean applyConfig(GokuIMUConfiguration config, double timeoutSeconds) {
    int hz = config.Features.DefaultUpdateFrequencyHz;
    if (hz < 1 || hz > 1000) {
      throw new IllegalArgumentException("DefaultUpdateFrequencyHz must be 1-1000");
    }
    if (m_sim != null) {
      if (!simConnected()) {
        return false;
      }
      m_sim.storeConfig(config);
      return true;
    }
    return command(Protocol.CMD_SET_CONFIG, Protocol.encodeConfig(config), timeoutSeconds);
  }

  /**
   * Reads the saved configuration into {@code config}. <b>Blocks</b> up to {@link
   * #DEFAULT_COMMAND_TIMEOUT}, plus the remaining time of another thread's refreshConfig already
   * running on this IMU (see {@link #refreshConfig(GokuIMUConfiguration, double)}).
   *
   * @return false if the IMU didn't reply
   */
  public boolean refreshConfig(GokuIMUConfiguration config) {
    return refreshConfig(config, DEFAULT_COMMAND_TIMEOUT);
  }

  /**
   * Reads the saved configuration; <b>blocks</b> up to {@code timeoutSeconds}. If another thread
   * is already in refreshConfig on this IMU, waits for it first (config replies can't be told
   * apart), so the worst case is the sum of both timeouts.
   */
  public boolean refreshConfig(GokuIMUConfiguration config, double timeoutSeconds) {
    if (m_sim != null) {
      if (!simConnected()) {
        return false;
      }
      m_sim.loadConfig(config);
      return true;
    }
    if (m_link == null) {
      return false;
    }
    synchronized (m_configLock) {
      CompletableFuture<byte[]> f = new CompletableFuture<>();
      m_configFuture = f;
      try {
        if (!m_link.sendFrame(Protocol.command(Protocol.CMD_GET_CONFIG, nextSeq(), null))) {
          return false;
        }
        byte[] p = f.get((long) (timeoutSeconds * 1000), TimeUnit.MILLISECONDS);
        if (p == null || p.length < Protocol.CONFIG_LEN) {
          return false;
        }
        Protocol.decodeConfig(p, config);
        return true;
      } catch (Exception e) {
        return false;
      } finally {
        m_configFuture = null;
      }
    }
  }

  /** Pigeon 2 style accessor: {@code imu.getConfigurator().apply(cfg)}. */
  public Configurator getConfigurator() {
    return new Configurator();
  }

  /** Pigeon 2 style configurator. Every method <b>blocks</b> like the GokuIMU method it calls. */
  public final class Configurator {
    private Configurator() {}

    /** <b>Blocks</b> up to {@link GokuIMU#DEFAULT_CONFIG_TIMEOUT}. */
    public boolean apply(GokuIMUConfiguration config) {
      return applyConfig(config);
    }

    /** <b>Blocks</b> up to {@code timeoutSeconds}. */
    public boolean apply(GokuIMUConfiguration config, double timeoutSeconds) {
      return applyConfig(config, timeoutSeconds);
    }

    /**
     * <b>Blocks</b> up to {@link GokuIMU#DEFAULT_COMMAND_TIMEOUT}, plus any other thread's
     * refresh already running on this IMU.
     */
    public boolean refresh(GokuIMUConfiguration config) {
      return refreshConfig(config);
    }

    /** <b>Blocks</b> up to {@link GokuIMU#DEFAULT_COMMAND_TIMEOUT}. */
    public boolean setYaw(double degrees) {
      return GokuIMU.this.setYaw(degrees);
    }

    /** <b>Blocks</b> up to {@code timeoutSeconds}. */
    public boolean setYaw(double degrees, double timeoutSeconds) {
      return GokuIMU.this.setYaw(degrees, timeoutSeconds);
    }

    /** <b>Blocks</b> up to {@link GokuIMU#DEFAULT_COMMAND_TIMEOUT}. */
    public boolean clearStickyFaults() {
      return GokuIMU.this.clearStickyFaults();
    }
  }

  // ---------------------------------------------------------------- device info (non-blocking)

  /**
   * True if frames are arriving (allows ~3 missed frames at the current stream rate). In
   * simulation, controlled by {@link GokuIMUSimState#setConnected}. Non-blocking.
   */
  public boolean isConnected() {
    if (m_sim != null) {
      return m_sim.isConnected();
    }
    double t = m_lastRxTime;
    double stale = Math.max(MIN_STALE_SECONDS, 3.5 * m_frameInterval);
    return t >= 0 && Timer.getTimestamp() - t < stale;
  }

  /**
   * Firmware version, serial number, etc. of the connected IMU, read automatically when it
   * connects. Null until then. Non-blocking.
   */
  public DeviceInfo getDeviceInfo() {
    if (m_sim != null) {
      return new DeviceInfo("sim", "SIM" + m_selector, "simulation", 0x47, Protocol.VERSION);
    }
    return m_info;
  }

  /** Firmware version "major.minor.patch", or null if not connected yet. Non-blocking. */
  public String getFirmwareVersion() {
    DeviceInfo i = getDeviceInfo();
    return i == null ? null : i.firmwareVersion();
  }

  /** USB serial number (the MCU unique ID), or null if not connected yet. Non-blocking. */
  public String getSerialNumber() {
    DeviceInfo i = getDeviceInfo();
    return i == null ? null : i.serialNumber();
  }

  /** Port the IMU is open on (e.g. /dev/ttyACM0), or null. Non-blocking. */
  public String getPort() {
    return m_link == null ? null : m_link.getOpenPath();
  }

  /**
   * All GokuIMUs the OS can see (Linux: attached boards; Windows: from the registry, may include
   * unplugged boards).
   *
   * <p><b>Blocks:</b> on Linux it reads a few sysfs files (instant). On Windows it runs {@code reg
   * query} twice, normally ~100 ms total; each run is abandoned after at most 3 s, so never more
   * than ~6 s. Call at startup or from a test, not in periodic code.
   */
  public static List<DevicePath> listDevices() {
    List<DevicePath> out = new ArrayList<>();
    for (String[] d : SerialLink.listDevices()) {
      out.add(new DevicePath(d[0], d[1]));
    }
    return out;
  }

  /** Simulation controls; null when talking to a real IMU. */
  public GokuIMUSimState getSimState() {
    return m_sim;
  }

  // ---------------------------------------------------------------- snapshot / gyro helpers

  /**
   * All values from one IMU frame, consistent with each other (use this for logging/replay, or
   * whenever you read several values together). Non-blocking. Allocates one small record per call;
   * for allocation-free reads in fast loops use {@link #readInputs(InputsBuffer)}.
   */
  public Inputs getInputs() {
    Snapshot snap = snapshot();
    Protocol.State s = snap.s();
    return new Inputs(
        snap.robotTime(), isConnected(), s.has(Protocol.FLAG_BIAS_VALID),
        s.has(Protocol.FLAG_MOVING), s.yaw(), s.pitch(), s.roll(), s.qw(), s.qx(), s.qy(), s.qz(),
        s.gyroX(), s.gyroY(), s.gyroZ(), worldRateZ(s), s.accelX(), s.accelY(), s.accelZ(),
        s.temperature(), s.flags() & FAULT_MASK, s.stickyFlags() & FAULT_MASK,
        s.flags() & ~FAULT_MASK);
  }

  /**
   * Allocation-free version of {@link #getInputs()}: copies every value from one IMU frame into a
   * caller-owned buffer (reuse one buffer per loop). Non-blocking.
   */
  public void readInputs(InputsBuffer out) {
    Snapshot snap = snapshot();
    Protocol.State s = snap.s();
    out.timestamp = snap.robotTime();
    out.connected = isConnected();
    out.calibrated = s.has(Protocol.FLAG_BIAS_VALID);
    out.moving = s.has(Protocol.FLAG_MOVING);
    out.yawDeg = s.yaw();
    out.pitchDeg = s.pitch();
    out.rollDeg = s.roll();
    out.qw = s.qw();
    out.qx = s.qx();
    out.qy = s.qy();
    out.qz = s.qz();
    out.rateXDeviceDps = s.gyroX();
    out.rateYDeviceDps = s.gyroY();
    out.rateZDeviceDps = s.gyroZ();
    out.yawRateWorldDps = worldRateZ(s);
    out.accelXG = s.accelX();
    out.accelYG = s.accelY();
    out.accelZG = s.accelZ();
    out.temperatureC = s.temperature();
    out.faultFlags = s.flags() & FAULT_MASK;
    out.stickyFaultFlags = s.stickyFlags() & FAULT_MASK;
    out.statusFlags = s.flags() & ~FAULT_MASK;
  }

  /** Mutable holder for {@link #readInputs(InputsBuffer)}. Same fields and units as {@link Inputs}. */
  public static final class InputsBuffer {
    public double timestamp;
    public boolean connected, calibrated, moving;
    public double yawDeg, pitchDeg, rollDeg;
    public double qw = 1, qx, qy, qz;
    public double rateXDeviceDps, rateYDeviceDps, rateZDeviceDps, yawRateWorldDps;
    public double accelXG, accelYG, accelZG;
    public double temperatureC;
    /** Active faults only; see {@link GokuIMU#FAULT_HARDWARE} etc. 0 means healthy. */
    public int faultFlags;
    /** Faults seen since the last clearStickyFaults(). */
    public int stickyFaultFlags;
    /** Non-fault status bits; see {@link GokuIMU#STATUS_CALIBRATED} etc. */
    public int statusFlags;
  }

  private static double worldRateZ(Protocol.State s) {
    // Z component of q * v_body * q^-1, without allocating.
    double w = s.qw(), x = s.qx(), y = s.qy(), z = s.qz();
    double vx = s.gyroX(), vy = s.gyroY(), vz = s.gyroZ();
    double tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
    return vz + w * tz + (x * ty - y * tx);
  }

  /** Continuous yaw in degrees, CCW positive (gyro-style name). Non-blocking. */
  public double getAngle() {
    return snapshot().s().yaw();
  }

  /**
   * Yaw rate about the vertical (world Z) axis in deg/s, CCW positive (gyro-style name). Note: the
   * old WPILib Gyro interface was CW positive; negate for legacy code. Non-blocking.
   */
  public double getRate() {
    return worldRates(snapshot().s())[2];
  }

  /** A supplier of {@link #getRotation2d()} for APIs that take one (e.g. drive commands). */
  public Supplier<Rotation2d> rotation2dSupplier() {
    return this::getRotation2d;
  }

  // ---------------------------------------------------------------- OnboardIMU style helpers
  // All non-blocking except resetYaw.

  /** Yaw as a Rotation2d (CCW positive). */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(snapshot().s().yaw());
  }

  /** Full 3D orientation. Yaw part is wrapped to +/-180. */
  public Rotation3d getRotation3d() {
    return new Rotation3d(getQuaternion());
  }

  public Quaternion getQuaternion() {
    Protocol.State s = snapshot().s();
    return new Quaternion(s.qw(), s.qx(), s.qy(), s.qz());
  }

  /** Continuous yaw in radians. */
  public double getYawRadians() {
    return Math.toRadians(snapshot().s().yaw());
  }

  /**
   * Same as setYaw(0). <b>Blocks</b> up to {@link #DEFAULT_COMMAND_TIMEOUT}.
   *
   * @return true if the IMU confirmed
   */
  public boolean resetYaw() {
    return setYaw(0);
  }

  /** Roll, radians. */
  public double getAngleX() {
    return Math.toRadians(snapshot().s().roll());
  }

  /** Pitch, radians. */
  public double getAngleY() {
    return Math.toRadians(snapshot().s().pitch());
  }

  /** Continuous yaw, radians. */
  public double getAngleZ() {
    return getYawRadians();
  }

  /** Robot-frame rates in rad/s. */
  public double getGyroRateX() {
    return Math.toRadians(snapshot().s().gyroX());
  }

  public double getGyroRateY() {
    return Math.toRadians(snapshot().s().gyroY());
  }

  public double getGyroRateZ() {
    return Math.toRadians(snapshot().s().gyroZ());
  }

  /** Robot-frame acceleration in m/s^2 (includes gravity). */
  public double getAccelX() {
    return snapshot().s().accelX() * 9.80665;
  }

  public double getAccelY() {
    return snapshot().s().accelY() * 9.80665;
  }

  public double getAccelZ() {
    return snapshot().s().accelZ() * 9.80665;
  }

  // ---------------------------------------------------------------- telemetry / lifecycle

  /**
   * Updates the dashboard alerts (disconnected, hardware fault, reboot while enabled). This runs
   * automatically 4 times a second; calling it yourself is optional. No I/O: three quick native
   * calls under a short internal lock (it may also wait the few microseconds it takes another
   * thread to finish {@link #close()} or an alert update).
   */
  public void updateAlerts() {
    synchronized (m_alertLock) {
      if (m_closed) {
        return;
      }
      boolean connected = isConnected();
      // In sim, read the fault switch directly instead of building a frame, so this background
      // task never advances simulation state.
      boolean hwFault =
          m_sim != null ? m_sim.hasHardwareFault() : m_snapshot.s().has(Protocol.FLAG_HW_FAULT);
      m_disconnectedAlert.set(!connected);
      m_faultAlert.set(connected && hwFault);
      m_bootAlert.set(m_bootDuringEnable);
    }
  }

  /** Alert executor tick: alerts, plus retrying the device-info request until it is answered. */
  private void updateAlertsQuietly() {
    try {
      updateAlerts();
      if (m_link != null && m_link.isOpen() && !m_closed) {
        if (m_info == null) {
          fireAndForget(Protocol.CMD_GET_INFO, null);
        }
        if (m_ratePending && m_requestedRateHz > 0) {
          sendRate(m_requestedRateHz);
        }
      }
    } catch (RuntimeException | UnsatisfiedLinkError e) {
      // natives unavailable (e.g. plain unit test); ignore
    }
  }

  /**
   * Logs one consistent frame. Uses the "Gyro" telemetry type with a "Value" key (yaw in degrees,
   * CCW positive) so dashboards show a gyro widget, plus every field of {@link Inputs}.
   */
  @Override
  public void logTo(TelemetryTable table) {
    Inputs in = getInputs();
    table.log("Value", in.yawDeg());
    in.logTo(table);
    table.log("Latency", Timer.getTimestamp() - in.timestamp());
    String port = getPort();
    if (port != null) {
      table.log("Port", port);
    }
  }

  @Override
  public String getTelemetryType() {
    return "Gyro";
  }

  /**
   * Stops the background thread and frees the sim device and alerts. Returns quickly: the port is
   * closed on a background thread; it may wait for one in-progress alert update (well under 1 ms)
   * on the internal alert lock.
   */
  @Override
  public void close() {
    synchronized (m_alertLock) {
      if (m_closed) {
        return;
      }
      m_closed = true; // the alert task checks this under the same lock
    }
    m_alertTask.cancel(false);
    if (m_link != null) {
      m_link.close();
    }
    if (m_sim != null) {
      m_sim.close();
    }
    synchronized (m_alertLock) {
      m_disconnectedAlert.close();
      m_faultAlert.close();
      m_bootAlert.close();
    }
  }
}
