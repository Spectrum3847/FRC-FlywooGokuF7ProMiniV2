package io.github.frcgokuimu;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Gs;

import org.wpilib.hardware.hal.SimBoolean;
import org.wpilib.hardware.hal.SimDevice;
import org.wpilib.hardware.hal.SimDevice.Direction;
import org.wpilib.hardware.hal.SimDouble;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;

/**
 * Simulation inputs for a {@link GokuIMU}, shaped like Phoenix 6's {@code Pigeon2SimState} and
 * WPILib's {@code OnboardIMUSim}. Values also show up (and are editable) in the WPILib simulation
 * GUI under "GokuIMU:&lt;name&gt;".
 *
 * <p><b>Raw vs. reported yaw:</b> physics sims set the <i>raw</i> yaw (the robot's true heading).
 * Robot code's {@link GokuIMU#setYaw} applies an offset on top, exactly like on hardware, so calling
 * {@code setRawYaw} every loop does not undo a {@code setYaw(0)} at the start of auto.
 *
 * <pre>{@code
 * // Differential drive
 * imu.getSimState().updateHeading(driveSim.getHeading(), omegaRadPerSec);
 * // Swerve / any 3D pose
 * imu.getSimState().update(new Rotation3d(heading), 0, 0, omegaRadPerSec);
 * }</pre>
 *
 * <p>No method does I/O or waits. All methods are {@code synchronized} on this object (shared
 * with the GokuIMU getters), so calls are serialized but only hold the lock for microseconds;
 * {@link #close()} also frees the native sim device. Accumulated angles integrate in simulation
 * time ({@link Timer#getTimestamp()}), so they follow paused/stepped sims, and keep integrating
 * while "unplugged" just like the real IMU (only the data robot code sees is frozen).
 */
public class GokuIMUSimState implements AutoCloseable {
  private final SimDevice m_device;
  private final SimDouble m_yaw, m_pitch, m_roll;
  private final SimDouble m_rateX, m_rateY, m_rateZ;
  private final SimDouble m_accX, m_accY, m_accZ;
  private final SimDouble m_temp;
  private final SimBoolean m_connected, m_hwFault, m_calibrated;

  private double m_yawOffsetDeg;
  private double m_accumX, m_accumY, m_accumZ;
  private double m_lastTime = Double.NaN;
  private long m_seq;
  private Protocol.State m_cached;
  private double m_cachedTime = Double.NaN;
  private final double m_startTime = Timer.getTimestamp();
  private GokuIMUConfiguration m_config = new GokuIMUConfiguration();

  GokuIMUSimState(String id) {
    String name = "GokuIMU:" + (id.isEmpty() ? "default" : id);
    m_device = SimDevice.create(name);
    if (m_device == null) {
      throw new IllegalStateException(
          "SimDevice \"" + name + "\" already exists. Each GokuIMU needs a unique serial/port"
              + " argument, and unit tests must close() the previous instance.");
    }
    m_yaw = m_device.createDouble("raw_yaw_deg", Direction.INPUT, 0);
    m_pitch = m_device.createDouble("pitch_deg", Direction.INPUT, 0);
    m_roll = m_device.createDouble("roll_deg", Direction.INPUT, 0);
    m_rateX = m_device.createDouble("rate_x_dps", Direction.INPUT, 0);
    m_rateY = m_device.createDouble("rate_y_dps", Direction.INPUT, 0);
    m_rateZ = m_device.createDouble("rate_z_dps", Direction.INPUT, 0);
    m_accX = m_device.createDouble("accel_x_g", Direction.INPUT, 0);
    m_accY = m_device.createDouble("accel_y_g", Direction.INPUT, 0);
    m_accZ = m_device.createDouble("accel_z_g", Direction.INPUT, 1);
    m_temp = m_device.createDouble("temperature_c", Direction.INPUT, 30);
    m_connected = m_device.createBoolean("connected", Direction.INPUT, true);
    m_hwFault = m_device.createBoolean("hardware_fault", Direction.INPUT, false);
    m_calibrated = m_device.createBoolean("calibrated", Direction.INPUT, true);
  }

  // ---------------------------------------------------------------- one-call physics hookup

  /**
   * Sets the full orientation and body-frame angular velocity in one call (recommended for physics
   * sims). Yaw from {@code pose} is unwrapped so the raw yaw stays continuous.
   *
   * @param pose robot orientation (roll, pitch, yaw)
   * @param wxRadPerSec body-frame rate about robot X
   * @param wyRadPerSec body-frame rate about robot Y
   * @param wzRadPerSec body-frame rate about robot Z
   */
  public synchronized void update(
      Rotation3d pose, double wxRadPerSec, double wyRadPerSec, double wzRadPerSec) {
    setContinuousRawYaw(Math.toDegrees(pose.getZ()));
    m_pitch.set(Math.toDegrees(pose.getY()));
    m_roll.set(Math.toDegrees(pose.getX()));
    m_rateX.set(Math.toDegrees(wxRadPerSec));
    m_rateY.set(Math.toDegrees(wyRadPerSec));
    m_rateZ.set(Math.toDegrees(wzRadPerSec));
  }

  /** 2D drivetrain hookup: heading (unwrapped automatically) and yaw rate. */
  public synchronized void updateHeading(Rotation2d heading, double yawRateRadPerSec) {
    setContinuousRawYaw(heading.getDegrees());
    m_rateZ.set(Math.toDegrees(yawRateRadPerSec));
  }

  private void setContinuousRawYaw(double wrappedDeg) {
    double raw = m_yaw.get();
    double delta = Math.IEEEremainder(wrappedDeg - raw, 360.0);
    m_yaw.set(raw + delta);
  }

  // ---------------------------------------------------------------- Pigeon2SimState style

  /** Sets the raw (true) continuous yaw. Robot code's setYaw offset is applied on top. */
  public synchronized void setRawYaw(Angle yaw) {
    m_yaw.set(yaw.in(Degrees));
  }

  /** Adds to the raw yaw (e.g. omega * dt from a drivetrain sim). */
  public synchronized void addYaw(Angle delta) {
    m_yaw.set(m_yaw.get() + delta.in(Degrees));
  }

  public synchronized void setPitch(Angle pitch) {
    m_pitch.set(pitch.in(Degrees));
  }

  public synchronized void setRoll(Angle roll) {
    m_roll.set(roll.in(Degrees));
  }

  /** Body-frame angular velocity about robot X. */
  public synchronized void setAngularVelocityX(AngularVelocity v) {
    m_rateX.set(v.in(DegreesPerSecond));
  }

  public synchronized void setAngularVelocityY(AngularVelocity v) {
    m_rateY.set(v.in(DegreesPerSecond));
  }

  public synchronized void setAngularVelocityZ(AngularVelocity v) {
    m_rateZ.set(v.in(DegreesPerSecond));
  }

  public synchronized void setAccelerationX(LinearAcceleration a) {
    m_accX.set(a.in(Gs));
  }

  public synchronized void setAccelerationY(LinearAcceleration a) {
    m_accY.set(a.in(Gs));
  }

  public synchronized void setAccelerationZ(LinearAcceleration a) {
    m_accZ.set(a.in(Gs));
  }

  // ---------------------------------------------------------------- OnboardIMUSim style (doubles)

  /** Roll, radians. */
  public synchronized void setAngleX(double rad) {
    m_roll.set(Math.toDegrees(rad));
  }

  /** Pitch, radians. */
  public synchronized void setAngleY(double rad) {
    m_pitch.set(Math.toDegrees(rad));
  }

  /** Raw continuous yaw, radians. */
  public synchronized void setAngleZ(double rad) {
    m_yaw.set(Math.toDegrees(rad));
  }

  public synchronized void setGyroRateX(double radPerSec) {
    m_rateX.set(Math.toDegrees(radPerSec));
  }

  public synchronized void setGyroRateY(double radPerSec) {
    m_rateY.set(Math.toDegrees(radPerSec));
  }

  public synchronized void setGyroRateZ(double radPerSec) {
    m_rateZ.set(Math.toDegrees(radPerSec));
  }

  /** Acceleration in m/s^2 (include +9.81 on Z for gravity when level). */
  public synchronized void setAccelX(double mps2) {
    m_accX.set(mps2 / 9.80665);
  }

  public synchronized void setAccelY(double mps2) {
    m_accY.set(mps2 / 9.80665);
  }

  public synchronized void setAccelZ(double mps2) {
    m_accZ.set(mps2 / 9.80665);
  }

  public synchronized void setTemperature(double celsius) {
    m_temp.set(celsius);
  }

  // ---------------------------------------------------------------- simulated faults

  /** Simulates unplugging the IMU: isConnected() goes false, commands fail, data freezes. */
  public synchronized void setConnected(boolean connected) {
    m_connected.set(connected);
  }

  /** Simulates the gyro chip failing (Fault_Hardware, dashboard alert). */
  public synchronized void setHardwareFault(boolean fault) {
    m_hwFault.set(fault);
  }

  /** Simulates the startup gyro calibration being incomplete (isCalibrating() returns true). */
  public synchronized void setCalibrated(boolean calibrated) {
    m_calibrated.set(calibrated);
  }

  // ---------------------------------------------------------------- used by GokuIMU

  synchronized boolean isConnected() {
    return m_connected.get();
  }

  synchronized boolean hasHardwareFault() {
    return m_hwFault.get();
  }

  /** Robot code setYaw: offset so the reported yaw equals {@code deg} now. */
  synchronized void setUserYaw(double deg) {
    m_yawOffsetDeg = deg - m_yaw.get();
    m_cachedTime = Double.NaN;
  }

  synchronized void storeConfig(GokuIMUConfiguration c) {
    m_config = new GokuIMUConfiguration();
    Protocol.decodeConfig(Protocol.encodeConfig(c), m_config);
  }

  synchronized void loadConfig(GokuIMUConfiguration into) {
    Protocol.decodeConfig(Protocol.encodeConfig(m_config), into);
  }

  /** Current simulated frame. Computed at most once per simulation timestamp. */
  synchronized Protocol.State state() {
    double now = Timer.getTimestamp();
    if (m_cached != null && now == m_cachedTime) {
      return m_cached; // same sim step
    }
    if (!Double.isNaN(m_lastTime)) {
      // Integrated body rates, like the real device. dt is clamped so a long pause in a
      // stepped sim doesn't produce a huge jump.
      double dt = Math.max(0, Math.min(now - m_lastTime, 0.1));
      m_accumX += m_rateX.get() * dt;
      m_accumY += m_rateY.get() * dt;
      m_accumZ += m_rateZ.get() * dt;
    }
    m_lastTime = now;
    if (m_cached != null && !m_connected.get()) {
      return m_cached; // "unplugged": the IMU keeps integrating but robot code sees frozen data
    }
    double yaw = m_yaw.get() + m_yawOffsetDeg, pitch = m_pitch.get(), roll = m_roll.get();
    var q =
        new Rotation3d(Math.toRadians(roll), Math.toRadians(pitch), Math.toRadians(yaw))
            .getQuaternion();
    long up = (long) ((now - m_startTime) * 1000);
    int flags = 0;
    if (m_calibrated.get()) {
      flags |= Protocol.FLAG_BIAS_VALID;
    }
    if (m_hwFault.get()) {
      flags |= Protocol.FLAG_HW_FAULT;
    }
    if (Math.abs(m_rateX.get()) + Math.abs(m_rateY.get()) + Math.abs(m_rateZ.get()) > 0.5) {
      flags |= Protocol.FLAG_MOVING;
    }
    m_cached =
        new Protocol.State(
            up * 1000, m_seq++, q.getW(), q.getX(), q.getY(), q.getZ(), yaw, pitch, roll,
            m_rateX.get(), m_rateY.get(), m_rateZ.get(), m_accX.get(), m_accY.get(),
            m_accZ.get(), m_accumX, m_accumY, m_accumZ, m_temp.get(), 0, flags,
            flags & Protocol.FLAG_HW_FAULT, 1000, 0, 0, 0, up);
    m_cachedTime = now;
    return m_cached;
  }

  @Override
  public synchronized void close() {
    m_device.close();
  }
}
