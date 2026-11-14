package io.github.frcgokuimu;

import java.util.function.DoubleFunction;
import java.util.function.ToDoubleFunction;
import org.wpilib.system.Timer;

/**
 * A single value reported by the IMU, shaped like Phoenix 6's {@code StatusSignal} so Pigeon 2 code
 * ports easily: {@code imu.getYaw().getValueAsDouble()}, {@code getValue()} for a units-typed
 * measure, and {@link #getTimestamp()} for latency compensation.
 *
 * <p>All signals come from the same streamed frame, so {@link #refresh()} is free and every signal
 * is always the newest data the IMU sent. Every method is non-blocking except {@link
 * #waitForUpdate}. To read several values from the same frame, use {@link GokuIMU#getInputs()}.
 *
 * @param <T> value type (a WPILib units Measure, Double, or Boolean)
 */
public final class ImuSignal<T> {
  private final GokuIMU m_imu;
  private final String m_name;
  private final String m_units;
  private final ToDoubleFunction<GokuIMU.Snapshot> m_extract;
  private final DoubleFunction<T> m_wrap;

  ImuSignal(
      GokuIMU imu,
      String name,
      String units,
      ToDoubleFunction<GokuIMU.Snapshot> extract,
      DoubleFunction<T> wrap) {
    m_imu = imu;
    m_name = name;
    m_units = units;
    m_extract = extract;
    m_wrap = wrap;
  }

  /** Value with units (Angle, AngularVelocity, ...), Double, or Boolean. */
  public T getValue() {
    return m_wrap.apply(getValueAsDouble());
  }

  /** Raw value in this signal's units (see {@link #getUnits()}); booleans are 0 or 1. */
  public double getValueAsDouble() {
    return m_extract.applyAsDouble(m_imu.snapshot());
  }

  /**
   * Estimated time the sample was measured, in the {@link Timer#getTimestamp()} time base
   * (seconds). Use with pose estimators.
   */
  public double getTimestamp() {
    return m_imu.snapshot().robotTime();
  }

  /** Seconds since this value was measured. */
  public double getAge() {
    return Timer.getTimestamp() - getTimestamp();
  }

  /** True if the IMU is connected and streaming. */
  public boolean isConnected() {
    return m_imu.isConnected();
  }

  /** No-op kept for Pigeon 2 compatibility; signals always hold the newest frame. */
  public ImuSignal<T> refresh() {
    return this;
  }

  /** <b>Blocks</b> the calling thread until a new frame arrives or the timeout passes. */
  public ImuSignal<T> waitForUpdate(double timeoutSeconds) {
    m_imu.waitForUpdate(timeoutSeconds);
    return this;
  }

  /** Sets the IMU stream rate (shared by all signals), 1-1000 Hz. Non-blocking. */
  public void setUpdateFrequency(double hz) {
    m_imu.setUpdateFrequency(hz);
  }

  public String getName() {
    return m_name;
  }

  public String getUnits() {
    return m_units;
  }

  @Override
  public String toString() {
    return m_name + ": " + getValueAsDouble() + " " + m_units;
  }

  /**
   * Latency-compensated value: {@code value + rate * age}. Same as Phoenix's
   * BaseStatusSignal.getLatencyCompensatedValue, e.g. (getYaw(), getAngularVelocityZWorld()).
   */
  public static double getLatencyCompensatedValue(ImuSignal<?> signal, ImuSignal<?> derivative) {
    return signal.getValueAsDouble() + derivative.getValueAsDouble() * signal.getAge();
  }
}
