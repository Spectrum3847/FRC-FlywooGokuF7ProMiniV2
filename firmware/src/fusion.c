#include "fusion.h"

#include <math.h>

#define PI_F 3.14159265358979f


static void quat_mul(const float a[4], const float b[4], float o[4]) {
  float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
  o[0] = w;
  o[1] = x;
  o[2] = y;
  o[3] = z;
}

static void quat_norm(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-9f) {
    q[0] = 1;
    q[1] = q[2] = q[3] = 0;
    return;
  }
  if (q[0] < 0) n = -n;  // keep w >= 0 for a canonical output
  for (int i = 0; i < 4; i++) q[i] /= n;
}

void quat_from_euler(float yaw, float pitch, float roll, float q[4]) {
  float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;
}

void quat_rotate(const float q[4], const float v[3], float out[3]) {
  // out = q * v * q^-1
  float w = q[0], x = q[1], y = q[2], z = q[3];
  float tx = 2.0f * (y * v[2] - z * v[1]);
  float ty = 2.0f * (z * v[0] - x * v[2]);
  float tz = 2.0f * (x * v[1] - y * v[0]);
  out[0] = v[0] + w * tx + (y * tz - z * ty);
  out[1] = v[1] + w * ty + (z * tx - x * tz);
  out[2] = v[2] + w * tz + (x * ty - y * tx);
}

void fusion_init_from_accel(fusion_t *f, const float a[3]) {
  float roll = atan2f(a[1], a[2]);
  float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
  quat_from_euler(0.0f, pitch, roll, f->q);
  f->yaw_turns = 0;
  f->last_yaw_rad = 0;
}

void fusion_update(fusion_t *f, const float g[3], const float a[3], float dt,
                   bool use_accel) {
  float wx = g[0], wy = g[1], wz = g[2];
  float *q = f->q;

  if (use_accel) {
    float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    // Only trust the accelerometer when it is close to 1 g (not accelerating).
    if (n > 0.9f && n < 1.1f) {
      float ax = a[0] / n, ay = a[1] / n, az = a[2] / n;
      // World "up" expressed in the body frame.
      float vx = 2.0f * (q[1] * q[3] - q[0] * q[2]);
      float vy = 2.0f * (q[0] * q[1] + q[2] * q[3]);
      float vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
      // Error axis is perpendicular to gravity, so it never changes heading.
      float ex = ay * vz - az * vy;
      float ey = az * vx - ax * vz;
      float ez = ax * vy - ay * vx;
      wx += f->kp * ex;
      wy += f->kp * ey;
      wz += f->kp * ez;
    }
  }

  // Exact rotation increment: dq = exp(0.5 * w * dt)
  float ang = sqrtf(wx * wx + wy * wy + wz * wz) * dt;
  float dq[4];
  if (ang > 1e-9f) {
    float s = sinf(ang * 0.5f) / (ang / dt);
    dq[0] = cosf(ang * 0.5f);
    dq[1] = wx * s;
    dq[2] = wy * s;
    dq[3] = wz * s;
  } else {
    dq[0] = 1.0f;
    dq[1] = 0.5f * wx * dt;
    dq[2] = 0.5f * wy * dt;
    dq[3] = 0.5f * wz * dt;
  }
  float nq[4];
  quat_mul(q, dq, nq);
  quat_norm(nq);
  for (int i = 0; i < 4; i++) q[i] = nq[i];

  // Track continuous yaw by unwrapping (1 kHz updates can't skip a half turn).
  float yaw, pitch, roll;
  fusion_euler(f, &yaw, &pitch, &roll);
  float d = yaw - f->last_yaw_rad;
  if (d > PI_F) f->yaw_turns--;
  else if (d < -PI_F) f->yaw_turns++;
  f->last_yaw_rad = yaw;
}

void fusion_euler(const fusion_t *f, float *yaw, float *pitch, float *roll) {
  const float *q = f->q;
  float w = q[0], x = q[1], y = q[2], z = q[3];
  *roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
  float sp = 2.0f * (w * y - z * x);
  if (sp > 1.0f) sp = 1.0f;
  if (sp < -1.0f) sp = -1.0f;
  *pitch = asinf(sp);
  *yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

double fusion_yaw_cont(const fusion_t *f) {
  return (double)f->yaw_turns * 6.283185307179586 + (double)f->last_yaw_rad;
}

void fusion_set_yaw(fusion_t *f, double yaw_rad) {
  float yaw, pitch, roll;
  fusion_euler(f, &yaw, &pitch, &roll);
  double turns = floor((yaw_rad + 3.141592653589793) / 6.283185307179586);
  float target = (float)(yaw_rad - turns * 6.283185307179586);  // in [-pi, pi)
  float d = target - yaw;
  float rz[4] = {cosf(d * 0.5f), 0, 0, sinf(d * 0.5f)};
  float nq[4];
  quat_mul(rz, f->q, nq);  // pre-multiply: rotate about world Z
  quat_norm(nq);
  for (int i = 0; i < 4; i++) f->q[i] = nq[i];
  fusion_euler(f, &yaw, &pitch, &roll);
  f->last_yaw_rad = yaw;
  // Pick the turn count so the continuous yaw lands on the requested value even
  // if float rounding put the wrapped yaw just across +/-pi.
  f->yaw_turns = (int32_t)lround((yaw_rad - (double)yaw) / 6.283185307179586);
}
