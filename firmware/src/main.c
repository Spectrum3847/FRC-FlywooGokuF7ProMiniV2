// GokuIMU firmware: turns a Flywoo Goku F7 Pro Mini V2 flight controller into
// a USB IMU for FRC robots (Systemcore). See protocol.h for the wire format.
#include <math.h>
#include <string.h>

#include "fusion.h"
#include "hw.h"
#include "icm42688.h"
#include "protocol.h"
#include "stm32f7xx.h"
#include "tusb.h"

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f

#define CFG_MAGIC 0x474B4931u  // "GKI1"

// No-motion calibration tuning.
#define STILL_BLOCK 500            // samples per stillness block (0.5 s)
#define STILL_GYRO_STD_DPS 0.50f   // per-axis std dev limit
#define STILL_ACCEL_STD_G 0.008f
#define STILL_BLOCKS_NEEDED 2      // consecutive still blocks before updating bias
#define STILL_MAX_BIAS_JUMP 1.0f   // dps; larger means we're actually turning slowly

typedef struct {
  uint32_t magic;
  uint32_t size;
  proto_config_t cfg;
  uint32_t crc;
} stored_config_t;

// ---------------- state ----------------
static proto_config_t s_cfg;
static bool s_cfg_default;
static fusion_t s_fusion;
static float s_mount_q[4];  // board -> robot
static float s_bias_board[3];
static bool s_bias_valid;
static uint16_t s_no_motion_count;
static uint16_t s_flags, s_sticky;
static double s_accum[3];  // double: keeps precision over many turns
static float s_gyro_robot[3], s_accel_robot[3];
static float s_temp_c;
static uint32_t s_last_sample_us, s_sample_ts;
static bool s_have_sample;
static bool s_hw_ok;
static uint8_t s_whoami;
static uint16_t s_rate_hz;
static uint16_t s_measured_rate;
static uint32_t s_seq;
static bool s_moved_since_boot;
static bool s_host_set_yaw;     // host set yaw since boot: don't auto-zero it
static bool s_reinit_fusion;    // mount pose changed: re-seed attitude, keep yaw
static int s_jump_blocks;       // consecutive quiet blocks sitting at one new offset
static float s_jump_ref[3];     // mean of the first such block

// stillness accumulators
static int s_blk_n;
static float s_blk_gs[3], s_blk_gss[3], s_blk_as[3], s_blk_ass[3];
static int s_still_blocks;
static bool s_force_cal;

// ---------------- helpers ----------------
static uint32_t crc32_sw(const void *data, uint32_t len) {
  const uint8_t *p = data;
  uint32_t c = 0xFFFFFFFFu;
  while (len--) {
    c ^= *p++;
    for (int i = 0; i < 8; i++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

static uint16_t crc16_update(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

static void apply_config(void) {
  quat_from_euler(s_cfg.mount_yaw_deg * DEG2RAD, s_cfg.mount_pitch_deg * DEG2RAD,
                  s_cfg.mount_roll_deg * DEG2RAD, s_mount_q);
  if (s_cfg.stream_rate_hz < 1 || s_cfg.stream_rate_hz > 1000) s_cfg.stream_rate_hz = 100;
}

static void config_defaults(void) {
  memset(&s_cfg, 0, sizeof(s_cfg));
  s_cfg.stream_rate_hz = 100;
}

static void config_load(void) {
  const stored_config_t *sc = flash_load_addr();
  if (sc->magic == CFG_MAGIC && sc->size == sizeof(proto_config_t) &&
      sc->crc == crc32_sw(&sc->cfg, sizeof(sc->cfg))) {
    s_cfg = sc->cfg;
    s_cfg_default = false;
  } else {
    config_defaults();
    s_cfg_default = true;
  }
  apply_config();
  s_rate_hz = s_cfg.stream_rate_hz;  // boot rate; CMD_SET_RATE overrides until reboot
}

static bool config_save(void) {
  stored_config_t sc = {CFG_MAGIC, sizeof(proto_config_t), s_cfg, 0};
  sc.crc = crc32_sw(&sc.cfg, sizeof(sc.cfg));
  bool ok = flash_store(&sc, sizeof(sc));
  if (ok) s_cfg_default = false;
  return ok;
}

// ---------------- USB framing ----------------
static void send_frame(uint8_t type, const void *payload, uint8_t len) {
  if (!tud_cdc_connected() && !tud_mounted()) return;
  uint32_t need = (uint32_t)len + 6;
  if (tud_cdc_write_available() < need) return;  // drop rather than block
  uint8_t hdr[4] = {PROTO_SYNC1, PROTO_SYNC2, type, len};
  uint16_t crc = 0xFFFF;
  crc = crc16_update(crc, type);
  crc = crc16_update(crc, len);
  const uint8_t *p = payload;
  for (uint8_t i = 0; i < len; i++) crc = crc16_update(crc, p[i]);
  uint8_t tail[2] = {(uint8_t)crc, (uint8_t)(crc >> 8)};
  tud_cdc_write(hdr, 4);
  if (len) tud_cdc_write(payload, len);
  tud_cdc_write(tail, 2);
  tud_cdc_write_flush();
}

static uint8_t s_cmd_seq;  // sequence byte of the command being handled

static void send_ack(uint8_t cmd, uint8_t status) {
  proto_ack_t a = {cmd, status, s_cmd_seq};
  send_frame(MSG_ACK, &a, sizeof(a));
}

static void send_info(void) {
  proto_info_t inf;
  memset(&inf, 0, sizeof(inf));
  inf.proto_version = PROTO_VERSION;
  inf.fw_major = FW_MAJOR;
  inf.fw_minor = FW_MINOR;
  inf.fw_patch = FW_PATCH;
  hw_uid(inf.uid);
  inf.imu_whoami = s_whoami;
  strncpy(inf.board, "FLYWOOF722PROV2", sizeof(inf.board) - 1);
  send_frame(MSG_INFO, &inf, sizeof(inf));
}

static void send_state(void) {
  proto_state_t st;
  float yaw, pitch, roll;
  fusion_euler(&s_fusion, &yaw, &pitch, &roll);
  st.timestamp_us = s_sample_ts;
  st.seq = s_seq++;
  memcpy(st.quat, s_fusion.q, sizeof(st.quat));
  st.yaw_deg = (float)(fusion_yaw_cont(&s_fusion) * 57.29577951308232);
  st.pitch_deg = pitch * RAD2DEG;
  st.roll_deg = roll * RAD2DEG;
  memcpy(st.gyro_dps, s_gyro_robot, sizeof(st.gyro_dps));
  memcpy(st.accel_g, s_accel_robot, sizeof(st.accel_g));
  for (int i = 0; i < 3; i++) st.accum_gyro_deg[i] = (float)s_accum[i];
  st.temp_c = s_temp_c;
  st.no_motion_count = s_no_motion_count;
  uint16_t flags = s_flags;
  if (!s_hw_ok) flags |= FLAG_HW_FAULT;
  if (s_bias_valid) flags |= FLAG_BIAS_VALID;
  if (g_hse_failed) flags |= FLAG_HSE_FAIL;
  if (g_millis < 3000) flags |= FLAG_BOOTING;
  if (s_cfg_default) flags |= FLAG_CFG_DEFAULT;
  s_sticky |= flags & (FLAG_HW_FAULT | FLAG_GYRO_SAT | FLAG_ACCEL_SAT | FLAG_HSE_FAIL |
                       FLAG_SAMPLE_MISSED);
  st.flags = flags;
  st.sticky_flags = s_sticky;
  st.sample_rate_hz = s_measured_rate;
  memcpy(st.gyro_bias_dps, s_bias_board, sizeof(st.gyro_bias_dps));
  st.uptime_ms = g_millis;
  send_frame(MSG_STATE, &st, sizeof(st));
  // Per-period flags reset once reported.
  s_flags &= (uint16_t)~(FLAG_GYRO_SAT | FLAG_ACCEL_SAT | FLAG_SAMPLE_MISSED);
}

static void reboot_after_flush(bool dfu) {
  uint32_t t0 = g_millis;
  while ((uint32_t)(g_millis - t0) < 100) tud_task();
  tud_disconnect();
  delay_ms(50);
  if (dfu) hw_reboot_to_dfu();
  hw_reboot();
}

static void handle_cmd(uint8_t type, const uint8_t *p, uint8_t len) {
  if (len < 1) {
    s_cmd_seq = 0;
    return send_ack(type, ACK_BAD_LEN);  // every command carries a sequence byte
  }
  s_cmd_seq = p[0];
  p++;
  len--;
  switch (type) {
    case CMD_SET_YAW: {
      if (len != 4) return send_ack(type, ACK_BAD_LEN);
      float v;
      memcpy(&v, p, 4);
      if (!isfinite(v) || fabsf(v) > 1e7f) return send_ack(type, ACK_BAD_ARG);
      fusion_set_yaw(&s_fusion, (double)v * 0.017453292519943295);
      s_host_set_yaw = true;
      return send_ack(type, ACK_OK);
    }
    case CMD_GET_INFO:
      return send_info();
    case CMD_GET_CONFIG:
      return send_frame(MSG_CONFIG, &s_cfg, sizeof(s_cfg));
    case CMD_SET_CONFIG: {
      if (len != sizeof(proto_config_t)) return send_ack(type, ACK_BAD_LEN);
      proto_config_t c;
      memcpy(&c, p, sizeof(c));
      if (c.stream_rate_hz < 1 || c.stream_rate_hz > 1000 || !isfinite(c.mount_yaw_deg) ||
          !isfinite(c.mount_pitch_deg) || !isfinite(c.mount_roll_deg) ||
          fabsf(c.gyro_trim_x) > 180 || fabsf(c.gyro_trim_y) > 180 || fabsf(c.gyro_trim_z) > 180)
        return send_ack(type, ACK_BAD_ARG);
      bool same = memcmp(&c, &s_cfg, sizeof(c)) == 0 && !s_cfg_default;
      bool mount_changed = c.mount_yaw_deg != s_cfg.mount_yaw_deg ||
                           c.mount_pitch_deg != s_cfg.mount_pitch_deg ||
                           c.mount_roll_deg != s_cfg.mount_roll_deg;
      // Only a changed default rate overrides a rate set with CMD_SET_RATE.
      if (c.stream_rate_hz != s_cfg.stream_rate_hz) s_rate_hz = c.stream_rate_hz;
      s_cfg = c;
      if (mount_changed) s_reinit_fusion = true;
      apply_config();
      if (!same && !config_save()) return send_ack(type, ACK_FLASH_FAIL);
      send_ack(type, ACK_OK);
      return send_frame(MSG_CONFIG, &s_cfg, sizeof(s_cfg));
    }
    case CMD_SET_RATE: {
      if (len != 2) return send_ack(type, ACK_BAD_LEN);
      uint16_t hz = (uint16_t)(p[0] | (p[1] << 8));
      if (hz < 1 || hz > 1000) return send_ack(type, ACK_BAD_ARG);
      s_rate_hz = hz;
      return send_ack(type, ACK_OK);
    }
    case CMD_CALIBRATE_GYRO:
      s_force_cal = true;
      s_still_blocks = 0;
      return send_ack(type, ACK_OK);
    case CMD_REBOOT:
      send_ack(type, ACK_OK);
      reboot_after_flush(false);
      return;
    case CMD_REBOOT_DFU:
      send_ack(type, ACK_OK);
      reboot_after_flush(true);
      return;
    case CMD_CLEAR_STICKY:
      s_sticky = 0;
      return send_ack(type, ACK_OK);
    case CMD_FACTORY_RESET:
      if (!flash_erase_store()) return send_ack(type, ACK_FLASH_FAIL);
      if (s_cfg.mount_yaw_deg != 0 || s_cfg.mount_pitch_deg != 0 || s_cfg.mount_roll_deg != 0)
        s_reinit_fusion = true;  // mount pose returns to zero
      config_defaults();
      s_cfg_default = true;
      apply_config();
      return send_ack(type, ACK_OK);
    default:
      return send_ack(type, ACK_UNKNOWN);
  }
}

// Byte-wise frame parser for host commands.
static void rx_byte(uint8_t b) {
  static uint8_t state, type, len, idx, buf[255];
  static uint16_t crc, rcrc;
  switch (state) {
    case 0: state = (b == PROTO_SYNC1) ? 1 : 0; break;
    case 1: state = (b == PROTO_SYNC2) ? 2 : (b == PROTO_SYNC1 ? 1 : 0); break;
    case 2: type = b; crc = crc16_update(0xFFFF, b); state = 3; break;
    case 3:
      len = b; crc = crc16_update(crc, b); idx = 0;
      state = len ? 4 : 5;
      break;
    case 4:
      buf[idx++] = b; crc = crc16_update(crc, b);
      if (idx >= len) state = 5;
      break;
    case 5: rcrc = b; state = 6; break;
    case 6:
      rcrc |= (uint16_t)b << 8;
      state = 0;
      if (rcrc == crc) handle_cmd(type, buf, len);
      break;
    default: state = 0;
  }
}

// ---------------- sensor processing ----------------
static void stillness_update(const float g[3], const float a[3]) {
  for (int i = 0; i < 3; i++) {
    s_blk_gs[i] += g[i];
    s_blk_gss[i] += g[i] * g[i];
    s_blk_as[i] += a[i];
    s_blk_ass[i] += a[i] * a[i];
  }
  if (++s_blk_n < STILL_BLOCK) return;

  bool still = true, jump = false;
  float mean[3];
  for (int i = 0; i < 3; i++) {
    mean[i] = s_blk_gs[i] / STILL_BLOCK;
    float gvar = s_blk_gss[i] / STILL_BLOCK - mean[i] * mean[i];
    float am = s_blk_as[i] / STILL_BLOCK;
    float avar = s_blk_ass[i] / STILL_BLOCK - am * am;
    if (gvar > STILL_GYRO_STD_DPS * STILL_GYRO_STD_DPS) still = false;
    if (avar > STILL_ACCEL_STD_G * STILL_ACCEL_STD_G) still = false;
    if (s_bias_valid && !s_force_cal && fabsf(mean[i] - s_bias_board[i]) > STILL_MAX_BIAS_JUMP)
      jump = true;
  }
  s_blk_n = 0;
  memset(s_blk_gs, 0, sizeof(s_blk_gs));
  memset(s_blk_gss, 0, sizeof(s_blk_gss));
  memset(s_blk_as, 0, sizeof(s_blk_as));
  memset(s_blk_ass, 0, sizeof(s_blk_ass));

  // A quiet board whose rate sits at one new offset (within 0.1 dps) for 10 s
  // is a real bias shift (e.g. temperature), not a turn: accept it. Anything
  // else with an offset is treated as motion.
  bool accept_jump = false;
  if (still && jump) {
    bool agrees = s_jump_blocks > 0;
    for (int i = 0; i < 3 && agrees; i++)
      if (fabsf(mean[i] - s_jump_ref[i]) > 0.1f) agrees = false;
    if (!agrees) {
      s_jump_blocks = 0;
      memcpy(s_jump_ref, mean, sizeof(mean));
    }
    if (++s_jump_blocks >= 20) {
      accept_jump = true;
      s_jump_blocks = 0;
    } else {
      still = false;
    }
  } else {
    s_jump_blocks = 0;
  }
  if (!still) {
    s_still_blocks = 0;
    s_moved_since_boot = true;
    s_flags |= FLAG_MOVING;
    return;
  }
  s_flags &= (uint16_t)~FLAG_MOVING;
  if (!accept_jump && ++s_still_blocks < STILL_BLOCKS_NEEDED) return;
  if ((s_cfg.features & FEAT_DISABLE_NO_MOTION_CAL) && s_bias_valid && !s_force_cal) return;

  bool first = !s_bias_valid || s_force_cal || accept_jump;
  for (int i = 0; i < 3; i++)
    s_bias_board[i] = first ? mean[i] : s_bias_board[i] + 0.3f * (mean[i] - s_bias_board[i]);
  if (!s_bias_valid && !s_moved_since_boot && !s_host_set_yaw) {
    // Robot has been still since power-up: discard drift accumulated before
    // the first bias estimate.
    fusion_set_yaw(&s_fusion, 0.0);
    memset(s_accum, 0, sizeof(s_accum));
  }
  s_bias_valid = true;
  s_force_cal = false;
  if (accept_jump) s_still_blocks = 0;
  s_no_motion_count++;
}

static void process_sample(const icm_raw_t *raw, uint32_t now) {
  // Sensor frame -> board frame. Gyro is mounted CW270 on this board:
  // board_x = -sensor_y, board_y = sensor_x (board X = arrow forward, Z up).
  float gs[3], as[3];
  for (int i = 0; i < 3; i++) {
    gs[i] = raw->gyro[i] / ICM_GYRO_LSB_PER_DPS;
    as[i] = raw->accel[i] / ICM_ACCEL_LSB_PER_G;
    if (raw->gyro[i] >= 32767 || raw->gyro[i] <= -32768) s_flags |= FLAG_GYRO_SAT;
    if (raw->accel[i] >= 32767 || raw->accel[i] <= -32768) s_flags |= FLAG_ACCEL_SAT;
  }
  float gb[3] = {-gs[1], gs[0], gs[2]};
  float ab[3] = {-as[1], as[0], as[2]};
  s_temp_c = icm_temp_c(raw->temp);

  stillness_update(gb, ab);

  float gbc[3] = {gb[0] - s_bias_board[0], gb[1] - s_bias_board[1], gb[2] - s_bias_board[2]};
  float gr[3], ar[3];
  quat_rotate(s_mount_q, gbc, gr);
  quat_rotate(s_mount_q, ab, ar);
  gr[0] *= 1.0f + s_cfg.gyro_trim_x / 360.0f;
  gr[1] *= 1.0f + s_cfg.gyro_trim_y / 360.0f;
  gr[2] *= 1.0f + s_cfg.gyro_trim_z / 360.0f;
  memcpy(s_gyro_robot, gr, sizeof(gr));
  memcpy(s_accel_robot, ar, sizeof(ar));

  if (s_reinit_fusion && s_have_sample) {
    double yaw = fusion_yaw_cont(&s_fusion);
    fusion_init_from_accel(&s_fusion, ar);
    fusion_set_yaw(&s_fusion, yaw);
    s_reinit_fusion = false;
  }
  if (!s_have_sample) {
    fusion_init_from_accel(&s_fusion, ar);
    s_have_sample = true;
    s_last_sample_us = now;
    s_sample_ts = now;
    return;
  }
  float dt = (float)(uint32_t)(now - s_last_sample_us) * 1e-6f;
  s_last_sample_us = now;
  s_sample_ts = now;
  if (dt > 0.0015f) s_flags |= FLAG_SAMPLE_MISSED;
  if (dt > 0.05f) dt = 0.05f;  // e.g. during a flash erase

  s_fusion.kp = (g_millis < 2000) ? 5.0f : 0.2f;
  float grad[3] = {gr[0] * DEG2RAD, gr[1] * DEG2RAD, gr[2] * DEG2RAD};
  fusion_update(&s_fusion, grad, ar, dt, !(s_cfg.features & FEAT_DISABLE_ACCEL_TILT));
  for (int i = 0; i < 3; i++) s_accum[i] += (double)gr[i] * dt;
}

// ---------------- main ----------------
void OTG_FS_IRQHandler(void) { tusb_int_handler(0, true); }

int main(void) {
  hw_init();
  config_load();

  s_whoami = icm_whoami();
  s_hw_ok = icm_init();
  s_whoami = icm_whoami();

  tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
  tusb_init(0, &dev_init);

  uint32_t next_frame = micros();
  uint32_t rate_t0 = g_millis, rate_n = 0;
  uint32_t last_drdy_ms = g_millis;

  while (1) {
    tud_task();

    while (tud_cdc_available()) {
      uint8_t buf[64];
      uint32_t n = tud_cdc_read(buf, sizeof(buf));
      for (uint32_t i = 0; i < n; i++) rx_byte(buf[i]);
    }

    if (s_hw_ok && imu_drdy_pin()) {
      icm_raw_t raw;
      uint32_t now = micros();
      if (icm_read(&raw)) {
        process_sample(&raw, now);
        rate_n++;
        last_drdy_ms = g_millis;
      }
    }
    // Data-ready stalled (or IMU never came up): re-init, retrying once a second.
    if ((uint32_t)(g_millis - last_drdy_ms) > (s_hw_ok ? 100u : 1000u)) {
      s_hw_ok = icm_init();
      s_whoami = icm_whoami();
      last_drdy_ms = g_millis;
    }

    if ((uint32_t)(g_millis - rate_t0) >= 1000) {
      s_measured_rate = (uint16_t)rate_n;
      rate_n = 0;
      rate_t0 = g_millis;
    }

    uint32_t now = micros();
    if ((int32_t)(now - next_frame) >= 0) {
      uint32_t period = 1000000u / s_rate_hz;
      next_frame += period;
      if ((int32_t)(now - next_frame) > (int32_t)period) next_frame = now + period;
      send_state();
    }

    // LED: slow blink when healthy and calibrated, fast when uncalibrated, solid on fault.
    uint32_t blink = !s_hw_ok ? 0 : (s_bias_valid ? 500 : 100);
    led_set(blink == 0 ? true : ((g_millis / blink) & 1));
  }
}
