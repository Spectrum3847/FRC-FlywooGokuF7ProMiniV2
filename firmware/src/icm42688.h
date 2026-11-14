#pragma once
#include <stdbool.h>
#include <stdint.h>

#define ICM42688_WHOAMI 0x47

// Full scale: gyro +/-2000 dps, accel +/-16 g, ODR 1 kHz.
#define ICM_GYRO_LSB_PER_DPS 16.4f
#define ICM_ACCEL_LSB_PER_G 2048.0f

typedef struct {
  int16_t accel[3];
  int16_t gyro[3];
  int16_t temp;
} icm_raw_t;

uint8_t icm_whoami(void);
bool icm_init(void);
// Reads INT_STATUS; returns true and fills raw if new data was ready.
bool icm_read(icm_raw_t *raw);
static inline float icm_temp_c(int16_t t) { return (float)t / 132.48f + 25.0f; }
