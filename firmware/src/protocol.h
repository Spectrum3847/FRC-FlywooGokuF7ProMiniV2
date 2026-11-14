// GokuIMU USB protocol. Shared definition between firmware and the vendordep
// (vendordep/src/main/java/.../Protocol.java mirrors this file; keep in sync).
//
// Every frame, in both directions:
//   0xA5 0x5A <type:u8> <len:u8> <payload: len bytes> <crc16:u16 LE>
// crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len, payload.
// Host -> device command payloads start with a sequence byte, followed by the
// command's arguments; the device echoes it in MSG_ACK so the host can match
// replies to requests (protocol v2).
// All multi-byte fields are little-endian. Floats are IEEE-754 binary32.
//
// Coordinate frame (after mount pose): FRC/WPILib convention, X forward,
// Y left, Z up, rotations counter-clockwise positive (right-hand rule).
#pragma once
#include <stdint.h>

#define PROTO_SYNC1 0xA5
#define PROTO_SYNC2 0x5A
#define PROTO_VERSION 2

// Device -> host
#define MSG_STATE 0x01      // periodic, payload = proto_state_t
#define MSG_INFO 0x02       // reply to CMD_GET_INFO, payload = proto_info_t
#define MSG_CONFIG 0x03     // reply to CMD_GET_CONFIG / CMD_SET_CONFIG, payload = proto_config_t
#define MSG_ACK 0x04        // payload = proto_ack_t

// Host -> device (all payloads: u8 seq, then the arguments listed)
#define CMD_SET_YAW 0x10          // args: float yaw_deg
#define CMD_GET_INFO 0x11         // no args
#define CMD_GET_CONFIG 0x12       // no args
#define CMD_SET_CONFIG 0x13       // args: proto_config_t (saved to flash)
#define CMD_SET_RATE 0x14         // args: u16 hz (1..1000, not persisted)
#define CMD_CALIBRATE_GYRO 0x15   // no args, forces bias re-cal (hold still)
#define CMD_REBOOT 0x16           // no args
#define CMD_REBOOT_DFU 0x17       // no args, jumps to STM32 ROM DFU bootloader
#define CMD_CLEAR_STICKY 0x18     // no args
#define CMD_FACTORY_RESET 0x19    // no args, erases saved config

#define ACK_OK 0
#define ACK_BAD_LEN 1
#define ACK_BAD_ARG 2
#define ACK_FLASH_FAIL 3
#define ACK_UNKNOWN 4

// Live fault / status flags (proto_state_t.flags)
#define FLAG_HW_FAULT (1u << 0)       // gyro not responding / wrong WHO_AM_I
#define FLAG_BIAS_VALID (1u << 1)     // a no-motion bias calibration has completed
#define FLAG_MOVING (1u << 2)         // not currently stationary
#define FLAG_GYRO_SAT (1u << 3)       // gyro saturated (> 2000 dps) in last period
#define FLAG_ACCEL_SAT (1u << 4)      // accel saturated (> 16 g) in last period
#define FLAG_HSE_FAIL (1u << 5)       // crystal failed to start; USB timing degraded
#define FLAG_SAMPLE_MISSED (1u << 6)  // sensor data-ready was late / skipped
#define FLAG_BOOTING (1u << 7)        // first seconds after boot, attitude converging
#define FLAG_CFG_DEFAULT (1u << 8)    // no saved config, defaults in use

// Feature bits (proto_config_t.features), mirror Pigeon2FeaturesConfigs
#define FEAT_DISABLE_NO_MOTION_CAL (1u << 0)
#define FEAT_DISABLE_ACCEL_TILT (1u << 1)  // pure gyro integration for pitch/roll

#pragma pack(push, 1)
typedef struct {
  uint32_t timestamp_us;   // device time of the newest sample
  uint32_t seq;            // increments per frame
  float quat[4];           // w, x, y, z: robot frame -> world frame
  float yaw_deg;           // continuous (not wrapped), like Pigeon2 getYaw()
  float pitch_deg;
  float roll_deg;
  float gyro_dps[3];       // robot frame, bias corrected
  float accel_g[3];        // robot frame, includes gravity
  float accum_gyro_deg[3]; // integrated robot-frame rates
  float temp_c;
  uint16_t no_motion_count;
  uint16_t flags;
  uint16_t sticky_flags;
  uint16_t sample_rate_hz; // measured sensor rate
  float gyro_bias_dps[3];
  uint32_t uptime_ms;
} proto_state_t;

typedef struct {
  uint8_t proto_version;
  uint8_t fw_major, fw_minor, fw_patch;
  uint32_t uid[3];
  uint8_t imu_whoami;
  uint8_t reserved[3];
  char board[24];
} proto_info_t;

typedef struct {
  float mount_yaw_deg, mount_pitch_deg, mount_roll_deg;
  float gyro_trim_x, gyro_trim_y, gyro_trim_z;  // degrees of error per rotation
  uint32_t features;
  uint16_t stream_rate_hz;  // default rate after boot
  uint16_t reserved;
} proto_config_t;

typedef struct {
  uint8_t cmd;
  uint8_t status;
  uint8_t seq;  // copied from the command
} proto_ack_t;
#pragma pack(pop)
