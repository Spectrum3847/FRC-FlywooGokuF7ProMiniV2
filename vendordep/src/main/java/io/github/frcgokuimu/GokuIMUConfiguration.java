package io.github.frcgokuimu;

/**
 * Persistent device configuration, stored in the IMU's flash. Shaped like Phoenix 6's {@code
 * Pigeon2Configuration} so Pigeon code ports with minimal edits.
 *
 * <p>Applying a configuration that differs from what is stored rewrites flash, which pauses the
 * IMU for about a second. Apply configs once at startup while the robot is still, not every loop.
 */
public class GokuIMUConfiguration {
  /** How the board is mounted on the robot. */
  public static class MountPoseConfigs {
    /**
     * Yaw of the board relative to the robot, degrees, CCW positive. If the arrow on the flight
     * controller points to the robot's left, this is 90.
     */
    public double MountPoseYaw = 0;
    /** Pitch of the board relative to the robot, degrees. */
    public double MountPosePitch = 0;
    /** Roll of the board relative to the robot, degrees. */
    public double MountPoseRoll = 0;

    public MountPoseConfigs withMountPoseYaw(double deg) {
      MountPoseYaw = deg;
      return this;
    }

    public MountPoseConfigs withMountPosePitch(double deg) {
      MountPosePitch = deg;
      return this;
    }

    public MountPoseConfigs withMountPoseRoll(double deg) {
      MountPoseRoll = deg;
      return this;
    }
  }

  /**
   * Gyro scale trim, in degrees of error per full rotation (same meaning as Pigeon 2's
   * GyroTrimConfigs). To calibrate Z: zero yaw, spin the robot exactly 10 turns either way, read
   * yaw, then add (3600 - |measured|) / 10 to the existing GyroScalarZ.
   */
  public static class GyroTrimConfigs {
    public double GyroScalarX = 0;
    public double GyroScalarY = 0;
    public double GyroScalarZ = 0;

    public GyroTrimConfigs withGyroScalarX(double v) {
      GyroScalarX = v;
      return this;
    }

    public GyroTrimConfigs withGyroScalarY(double v) {
      GyroScalarY = v;
      return this;
    }

    public GyroTrimConfigs withGyroScalarZ(double v) {
      GyroScalarZ = v;
      return this;
    }
  }

  /** Feature toggles. */
  public static class FeaturesConfigs {
    /** Disable the automatic gyro bias calibration that runs whenever the robot is still. */
    public boolean DisableNoMotionCalibration = false;
    /** Disable gravity-based pitch/roll correction (pure gyro integration). */
    public boolean DisableAccelTiltCorrection = false;
    /** Stream rate the device boots with, 1-1000 Hz. */
    public int DefaultUpdateFrequencyHz = 100;

    public FeaturesConfigs withDisableNoMotionCalibration(boolean v) {
      DisableNoMotionCalibration = v;
      return this;
    }

    public FeaturesConfigs withDisableAccelTiltCorrection(boolean v) {
      DisableAccelTiltCorrection = v;
      return this;
    }

    public FeaturesConfigs withDefaultUpdateFrequencyHz(int hz) {
      DefaultUpdateFrequencyHz = hz;
      return this;
    }
  }

  public MountPoseConfigs MountPose = new MountPoseConfigs();
  public GyroTrimConfigs GyroTrim = new GyroTrimConfigs();
  public FeaturesConfigs Features = new FeaturesConfigs();

  public GokuIMUConfiguration withMountPose(MountPoseConfigs c) {
    MountPose = c;
    return this;
  }

  public GokuIMUConfiguration withGyroTrim(GyroTrimConfigs c) {
    GyroTrim = c;
    return this;
  }

  public GokuIMUConfiguration withFeatures(FeaturesConfigs c) {
    Features = c;
    return this;
  }

  @Override
  public String toString() {
    return String.format(
        "GokuIMUConfiguration{mount=(y %.2f, p %.2f, r %.2f), trim=(%.3f, %.3f, %.3f),"
            + " noMotionCalDisabled=%b, tiltDisabled=%b, rate=%d}",
        MountPose.MountPoseYaw, MountPose.MountPosePitch, MountPose.MountPoseRoll,
        GyroTrim.GyroScalarX, GyroTrim.GyroScalarY, GyroTrim.GyroScalarZ,
        Features.DisableNoMotionCalibration, Features.DisableAccelTiltCorrection,
        Features.DefaultUpdateFrequencyHz);
  }
}
