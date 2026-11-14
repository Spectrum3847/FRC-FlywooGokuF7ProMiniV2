// Low-level hardware for the Flywoo Goku F7 Pro Mini V2 (FLYWOOF722PROV2).
// MCU STM32F722RET6, 8 MHz HSE. ICM-42688-P on SPI1 (PA5/PA6/PA7, CS PA4,
// INT1 PC3), second gyro footprint CS PB2 (unpopulated, held high).
// Status LED PC15 (active low).
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define FW_MAJOR 0
#define FW_MINOR 2
#define FW_PATCH 0

extern volatile uint32_t g_millis;
extern bool g_hse_failed;

void hw_init(void);
uint32_t micros(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void led_set(bool on);

void spi_cs(bool active);
uint8_t spi_xfer(uint8_t b);

bool imu_drdy_pin(void);

void hw_uid(uint32_t out[3]);
void hw_reboot(void);
void hw_reboot_to_dfu(void);

// Config storage in flash sector 7 (0x08060000, 128 KiB).
bool flash_store(const void *data, uint32_t len);
const void *flash_load_addr(void);
bool flash_erase_store(void);
