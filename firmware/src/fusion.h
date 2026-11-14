// 6-axis attitude filter (Mahony-style complementary filter).
// Pitch/roll are corrected from gravity; yaw is pure gyro integration
// (there is no magnetometer, same as Pigeon 2 with the compass disabled).
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float q[4];  // w x y z, body -> world
  float kp;    // accel correction gain (rad/s per rad)
  int32_t yaw_turns;  // whole turns, so continuous yaw keeps full precision
  float last_yaw_rad; // wrapped yaw at the last update
} fusion_t;

void fusion_init_from_accel(fusion_t *f, const float accel_g[3]);
// gyro in rad/s (bias corrected), accel in g. use_accel=false skips tilt correction.
void fusion_update(fusion_t *f, const float gyro_rads[3], const float accel_g[3],
                   float dt, bool use_accel);
void fusion_euler(const fusion_t *f, float *yaw, float *pitch, float *roll);
// Sets the continuous yaw and rotates the quaternion about world Z to match.
void fusion_set_yaw(fusion_t *f, double yaw_rad);
// Continuous (unwrapped) yaw in radians.
double fusion_yaw_cont(const fusion_t *f);

void quat_from_euler(float yaw, float pitch, float roll, float q[4]);
void quat_rotate(const float q[4], const float v[3], float out[3]);
