#include "icm42688.h"

#include "hw.h"

#define REG_DEVICE_CONFIG 0x11
#define REG_INT_CONFIG 0x14
#define REG_TEMP_DATA1 0x1D
#define REG_INT_STATUS 0x2D
#define REG_PWR_MGMT0 0x4E
#define REG_GYRO_CONFIG0 0x4F
#define REG_ACCEL_CONFIG0 0x50
#define REG_GYRO_ACCEL_CONFIG0 0x52
#define REG_INT_CONFIG1 0x64
#define REG_INT_SOURCE0 0x65
#define REG_WHO_AM_I 0x75
#define REG_BANK_SEL 0x76

static void wr(uint8_t reg, uint8_t v) {
  spi_cs(true);
  spi_xfer(reg & 0x7F);
  spi_xfer(v);
  spi_cs(false);
  delay_us(10);
}

static uint8_t rd(uint8_t reg) {
  spi_cs(true);
  spi_xfer(reg | 0x80);
  uint8_t v = spi_xfer(0);
  spi_cs(false);
  return v;
}

uint8_t icm_whoami(void) { return rd(REG_WHO_AM_I); }

bool icm_init(void) {
  wr(REG_BANK_SEL, 0);
  wr(REG_DEVICE_CONFIG, 0x01);  // soft reset
  delay_ms(5);
  wr(REG_BANK_SEL, 0);
  if (icm_whoami() != ICM42688_WHOAMI) return false;

  wr(REG_PWR_MGMT0, 0x0F);  // gyro + accel low-noise mode
  delay_ms(1);
  wr(REG_GYRO_CONFIG0, (0u << 5) | 0x06);   // 2000 dps, 1 kHz
  wr(REG_ACCEL_CONFIG0, (0u << 5) | 0x06);  // 16 g, 1 kHz
  wr(REG_GYRO_ACCEL_CONFIG0, 0x11);         // UI LPF BW = ODR/4 (both)
  wr(REG_INT_CONFIG, 0x07);    // INT1 latched, push-pull, active high
  wr(REG_INT_CONFIG1, 0x00);   // required for proper INT operation at >=4 kHz too
  wr(REG_INT_SOURCE0, 0x08);   // UI data ready -> INT1
  delay_ms(50);                // gyro start-up
  return icm_whoami() == ICM42688_WHOAMI;
}

bool icm_read(icm_raw_t *raw) {
  uint8_t b[15];
  spi_cs(true);
  spi_xfer(REG_INT_STATUS | 0x80);
  uint8_t st = spi_xfer(0);
  spi_cs(false);
  if (!(st & 0x08)) return false;

  spi_cs(true);
  spi_xfer(REG_TEMP_DATA1 | 0x80);
  for (int i = 0; i < 14; i++) b[i] = spi_xfer(0);
  spi_cs(false);

  raw->temp = (int16_t)((b[0] << 8) | b[1]);
  for (int i = 0; i < 3; i++) {
    raw->accel[i] = (int16_t)((b[2 + i * 2] << 8) | b[3 + i * 2]);
    raw->gyro[i] = (int16_t)((b[8 + i * 2] << 8) | b[9 + i * 2]);
  }
  return true;
}
