package io.github.frcgokuimu;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Desk test of the full GokuIMU class with WPILib desktop natives (simulation HAL).
 * Usage: java -Djava.library.path=build/natives ... FullTest [COMx]
 */
public class FullTest {
  static int failures;

  static void check(String what, boolean ok) {
    System.out.println((ok ? "PASS " : "FAIL ") + what);
    if (!ok) {
      failures++;
    }
  }

  static boolean near(double a, double b, double tol) {
    return Math.abs(a - b) <= tol;
  }

  public static void main(String[] args) throws Exception {
    HAL.initialize();
    simTests();
    if (args.length > 0) {
      hardwareTests(args[0]);
    }
    System.out.println(failures == 0 ? "ALL PASSED" : failures + " FAILED");
    System.exit(failures == 0 ? 0 : 1);
  }

  static void simTests() {
    System.out.println("--- simulation ---");
    GokuIMU imu = new GokuIMU("", GokuIMU.Mode.SIMULATION);
    GokuIMUSimState sim = imu.getSimState();
    check("sim connected", imu.isConnected() && imu.waitForConnection(0.1));
    sim.updateHeading(Rotation2d.fromDegrees(170), 0);
    sim.updateHeading(Rotation2d.fromDegrees(-170), 0); // crosses 180: raw should be 190
    check("raw yaw unwraps across 180", near(imu.getAngle(), 190, 1e-6));
    check("setYaw(0) in sim", imu.setYaw(0));
    sim.updateHeading(Rotation2d.fromDegrees(-160), Math.toRadians(30)); // +10 deg raw
    check("setYaw survives physics updates (yaw=10)", near(imu.getAngle(), 10, 1e-6));
    check("getRotation2d matches", near(imu.getRotation2d().getDegrees(), 10, 1e-6));
    check("getRate = 30 deg/s", near(imu.getRate(), 30, 1e-6));
    check(
        "quaternion yaw matches", near(Math.toDegrees(imu.getRotation3d().getZ()), 10, 1e-6));
    GokuIMU.Inputs in = imu.getInputs();
    check("inputs consistent", near(in.yawDeg(), 10, 1e-6) && in.connected());
    check("calibrated is a status bit, not a fault",
        in.faultFlags() == 0 && (in.statusFlags() & GokuIMU.STATUS_CALIBRATED) != 0);
    var buf = new GokuIMU.InputsBuffer();
    imu.readInputs(buf);
    check("readInputs matches getInputs",
        near(buf.yawDeg, in.yawDeg(), 1e-9) && near(buf.yawRateWorldDps, in.yawRateWorldDps(), 1e-9)
            && buf.faultFlags == 0);
    sim.setCalibrated(false);
    check("isCalibrating when uncalibrated", imu.isCalibrating());
    check("waitForCalibration times out", !imu.waitForCalibration(0.05));
    long w0 = System.nanoTime();
    boolean waited = imu.waitForCalibration(0.3);
    double waitedS = (System.nanoTime() - w0) / 1e9;
    check(String.format("sim waitForCalibration sleeps for its timeout (%.2f s)", waitedS),
        !waited && waitedS >= 0.28 && waitedS < 0.6);
    sim.setCalibrated(true);
    check("calibrated", !imu.isCalibrating() && imu.waitForCalibration(0.1));
    sim.setHardwareFault(true);
    check("hardware fault signal", imu.getFault_Hardware().getValue());
    sim.setHardwareFault(false);
    sim.setConnected(false);
    check("disconnected", !imu.isConnected() && imu.getFault_Hardware().getValue());
    check("setYaw fails when disconnected", !imu.setYaw(5));
    check("waitForConnection times out", !imu.waitForConnection(0.05));
    sim.setConnected(true);
    var cfg = new GokuIMUConfiguration();
    cfg.MountPose.MountPoseYaw = 90;
    check("applyConfig sim", imu.getConfigurator().apply(cfg));
    var back = new GokuIMUConfiguration();
    check("refreshConfig sim", imu.refreshConfig(back) && back.MountPose.MountPoseYaw == 90);
    check("device info sim", imu.getDeviceInfo() != null);
    sim.addYaw(Degrees.of(5));
    check("addYaw", near(imu.getYaw().getValueAsDouble(), 15, 1e-6));
    boolean threw = false;
    try {
      new GokuIMU("", GokuIMU.Mode.SIMULATION);
    } catch (IllegalStateException e) {
      threw = e.getMessage().contains("already exists");
    }
    check("duplicate instance gives clear error", threw);
    imu.close();
    GokuIMU again = new GokuIMU("", GokuIMU.Mode.SIMULATION);
    check("re-create after close", again.isConnected());
    again.close();
  }

  static void hardwareTests(String port) throws Exception {
    System.out.println("--- hardware ---");
    System.out.println("listDevices: " + GokuIMU.listDevices());
    // Auto-discovery (Windows registry) instead of an explicit port.
    GokuIMU imu = new GokuIMU("", GokuIMU.Mode.HARDWARE);
    check("auto-discovered and connected", imu.waitForConnection(5));
    System.out.println("port " + imu.getPort());
    Thread.sleep(300);
    System.out.println("device info: " + imu.getDeviceInfo());
    check("device info cached", imu.getDeviceInfo() != null);
    check("firmware is 0.2.x", imu.getFirmwareVersion() != null && imu.getFirmwareVersion().startsWith("0.2"));
    check("waitForCalibration", imu.waitForCalibration(5));
    for (double v : new double[] {45, -123.5, 3600.25, 0}) {
      long t0 = System.nanoTime();
      boolean ok = imu.setYaw(v);
      double ms = (System.nanoTime() - t0) / 1e6;
      imu.getYaw().waitForUpdate(1.0);
      imu.getYaw().waitForUpdate(1.0);
      check(
          String.format("setYaw(%s) ok in %.1f ms, reads %.3f", v, ms, imu.getAngle()),
          ok && near(imu.getAngle(), v, 0.2));
    }
    // Many commands back to back: every one must be confirmed (sequence matching).
    int good = 0;
    for (int i = 0; i < 50; i++) {
      if (imu.setYaw(i)) {
        good++;
      }
    }
    check("50 back-to-back setYaw all confirmed (" + good + ")", good == 50);
    imu.setUpdateFrequency(50);
    Thread.sleep(500);
    var cfg = new GokuIMUConfiguration();
    check("refreshConfig", imu.refreshConfig(cfg));
    check("applyConfig unchanged (no flash write)", imu.applyConfig(cfg));
    double t0 = System.nanoTime();
    int frames = 0;
    while ((System.nanoTime() - t0) < 2e9) {
      if (imu.getYaw().waitForUpdate(0.5) != null) {
        frames++;
      }
    }
    check("rate stays ~50 Hz after applyConfig (" + frames / 2 + "/s)", frames / 2 < 80);
    imu.setUpdateFrequency(100);
    // A setYaw issued while a config save (flash erase, ~1 s) is in flight must honour its own
    // 0.25 s timeout instead of waiting for the save.
    var changed = new GokuIMUConfiguration();
    changed.MountPose.MountPoseRoll = 0.5;
    var saver = new Thread(() -> imu.applyConfig(changed));
    saver.start();
    Thread.sleep(20);
    long y0 = System.nanoTime();
    imu.setYaw(0);
    double yawMs = (System.nanoTime() - y0) / 1e6;
    saver.join();
    check(String.format("setYaw during config save returns within its timeout (%.0f ms)", yawMs),
        yawMs < 300);
    check("restore default config", imu.applyConfig(new GokuIMUConfiguration()));
    imu.waitForConnection(3);
    check("connected", imu.isConnected());
    System.out.println("inputs: " + imu.getInputs());
    check("setYaw(0)", imu.setYaw(0));
    imu.close();
  }
}
