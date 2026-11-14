package io.github.frcgokuimu;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/** Wire protocol shared with the firmware. Mirrors firmware/src/protocol.h. */
final class Protocol {
  private Protocol() {}

  static final int SYNC1 = 0xA5;
  static final int SYNC2 = 0x5A;
  static final int VERSION = 2;

  static final int MSG_STATE = 0x01;
  static final int MSG_INFO = 0x02;
  static final int MSG_CONFIG = 0x03;
  static final int MSG_ACK = 0x04;

  static final int CMD_SET_YAW = 0x10;
  static final int CMD_GET_INFO = 0x11;
  static final int CMD_GET_CONFIG = 0x12;
  static final int CMD_SET_CONFIG = 0x13;
  static final int CMD_SET_RATE = 0x14;
  static final int CMD_CALIBRATE_GYRO = 0x15;
  static final int CMD_REBOOT = 0x16;
  static final int CMD_REBOOT_DFU = 0x17;
  static final int CMD_CLEAR_STICKY = 0x18;
  static final int CMD_FACTORY_RESET = 0x19;

  static final int ACK_OK = 0;

  static final int FLAG_HW_FAULT = 1 << 0;
  static final int FLAG_BIAS_VALID = 1 << 1;
  static final int FLAG_MOVING = 1 << 2;
  static final int FLAG_GYRO_SAT = 1 << 3;
  static final int FLAG_ACCEL_SAT = 1 << 4;
  static final int FLAG_HSE_FAIL = 1 << 5;
  static final int FLAG_SAMPLE_MISSED = 1 << 6;
  static final int FLAG_BOOTING = 1 << 7;
  static final int FLAG_CFG_DEFAULT = 1 << 8;

  static final int STATE_LEN = 4 + 4 + 16 + 12 + 36 + 4 + 8 + 12 + 4;
  static final int CONFIG_LEN = 6 * 4 + 4 + 2 + 2;

  static int crc16(byte[] data, int off, int len) {
    int crc = 0xFFFF;
    for (int i = off; i < off + len; i++) {
      crc ^= (data[i] & 0xFF) << 8;
      for (int b = 0; b < 8; b++) {
        crc = (crc & 0x8000) != 0 ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
      }
    }
    return crc;
  }

  /** Builds a host command: payload is the sequence byte followed by the arguments. */
  static byte[] command(int type, int seq, byte[] args) {
    int n = args == null ? 0 : args.length;
    byte[] p = new byte[n + 1];
    p[0] = (byte) seq;
    if (n > 0) {
      System.arraycopy(args, 0, p, 1, n);
    }
    return frame(type, p);
  }

  static byte[] frame(int type, byte[] payload) {
    int n = payload == null ? 0 : payload.length;
    byte[] f = new byte[n + 6];
    f[0] = (byte) SYNC1;
    f[1] = (byte) SYNC2;
    f[2] = (byte) type;
    f[3] = (byte) n;
    if (n > 0) {
      System.arraycopy(payload, 0, f, 4, n);
    }
    int crc = crc16(f, 2, n + 2);
    f[n + 4] = (byte) crc;
    f[n + 5] = (byte) (crc >> 8);
    return f;
  }

  static ByteBuffer le(byte[] b) {
    return ByteBuffer.wrap(b).order(ByteOrder.LITTLE_ENDIAN);
  }

  /** One decoded MSG_STATE frame. Angles in degrees, rates in deg/s, accel in g. */
  record State(
      long deviceTimestampUs,
      long seq,
      double qw,
      double qx,
      double qy,
      double qz,
      double yaw,
      double pitch,
      double roll,
      double gyroX,
      double gyroY,
      double gyroZ,
      double accelX,
      double accelY,
      double accelZ,
      double accumX,
      double accumY,
      double accumZ,
      double temperature,
      int noMotionCount,
      int flags,
      int stickyFlags,
      int sampleRateHz,
      double biasX,
      double biasY,
      double biasZ,
      long uptimeMs) {

    static State parse(byte[] p) {
      ByteBuffer b = le(p);
      return new State(
          Integer.toUnsignedLong(b.getInt()),
          Integer.toUnsignedLong(b.getInt()),
          b.getFloat(), b.getFloat(), b.getFloat(), b.getFloat(),
          b.getFloat(), b.getFloat(), b.getFloat(),
          b.getFloat(), b.getFloat(), b.getFloat(),
          b.getFloat(), b.getFloat(), b.getFloat(),
          b.getFloat(), b.getFloat(), b.getFloat(),
          b.getFloat(),
          Short.toUnsignedInt(b.getShort()),
          Short.toUnsignedInt(b.getShort()),
          Short.toUnsignedInt(b.getShort()),
          Short.toUnsignedInt(b.getShort()),
          b.getFloat(), b.getFloat(), b.getFloat(),
          Integer.toUnsignedLong(b.getInt()));
    }

    boolean has(int flag) {
      return (flags & flag) != 0;
    }

    boolean sticky(int flag) {
      return (stickyFlags & flag) != 0;
    }
  }

  static final State ZERO =
      new State(0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0);

  static byte[] encodeConfig(GokuIMUConfiguration c) {
    ByteBuffer b = ByteBuffer.allocate(CONFIG_LEN).order(ByteOrder.LITTLE_ENDIAN);
    b.putFloat((float) c.MountPose.MountPoseYaw);
    b.putFloat((float) c.MountPose.MountPosePitch);
    b.putFloat((float) c.MountPose.MountPoseRoll);
    b.putFloat((float) c.GyroTrim.GyroScalarX);
    b.putFloat((float) c.GyroTrim.GyroScalarY);
    b.putFloat((float) c.GyroTrim.GyroScalarZ);
    int features = 0;
    if (c.Features.DisableNoMotionCalibration) {
      features |= 1;
    }
    if (c.Features.DisableAccelTiltCorrection) {
      features |= 2;
    }
    b.putInt(features);
    b.putShort((short) c.Features.DefaultUpdateFrequencyHz);
    b.putShort((short) 0);
    return b.array();
  }

  static void decodeConfig(byte[] p, GokuIMUConfiguration c) {
    ByteBuffer b = le(p);
    c.MountPose.MountPoseYaw = b.getFloat();
    c.MountPose.MountPosePitch = b.getFloat();
    c.MountPose.MountPoseRoll = b.getFloat();
    c.GyroTrim.GyroScalarX = b.getFloat();
    c.GyroTrim.GyroScalarY = b.getFloat();
    c.GyroTrim.GyroScalarZ = b.getFloat();
    int features = b.getInt();
    c.Features.DisableNoMotionCalibration = (features & 1) != 0;
    c.Features.DisableAccelTiltCorrection = (features & 2) != 0;
    c.Features.DefaultUpdateFrequencyHz = Short.toUnsignedInt(b.getShort());
  }
}
