#include "hw.h"

#include <string.h>

#include "stm32f7xx.h"

#define DFU_MAGIC 0xDF00B007u
#define SYSMEM_BASE 0x1FF00000u
#define UID_BASE_ADDR 0x1FF07A10u
#define CFG_SECTOR 7u
#define CFG_ADDR 0x08060000u

uint32_t SystemCoreClock = 16000000u;
volatile uint32_t g_millis;
bool g_hse_failed;

// Survives a software reset: not zeroed by the startup code.
__attribute__((section(".noinit"))) static volatile uint32_t s_boot_magic;

// Called from the startup code before .data/.bss init and before main().
void SystemInit(void) {
  SCB->CPACR |= (0xFu << 20);  // CP10/CP11 full access (FPU)
  __DSB();
  __ISB();
  if (s_boot_magic == DFU_MAGIC) {
    s_boot_magic = 0;
    const uint32_t *sys = (const uint32_t *)SYSMEM_BASE;
    __set_MSP(sys[0]);
    ((void (*)(void))sys[1])();
    while (1) {
    }
  }
}

void SysTick_Handler(void) { g_millis++; }

static void gpio_mode(GPIO_TypeDef *g, int pin, uint32_t mode, uint32_t af,
                      uint32_t speed) {
  g->MODER = (g->MODER & ~(3u << (pin * 2))) | (mode << (pin * 2));
  g->OSPEEDR = (g->OSPEEDR & ~(3u << (pin * 2))) | (speed << (pin * 2));
  if (mode == 2) {
    volatile uint32_t *afr = &g->AFR[pin >> 3];
    int sh = (pin & 7) * 4;
    *afr = (*afr & ~(0xFu << sh)) | (af << sh);
  }
}

static void clock_init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  (void)RCC->APB1ENR;
  PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS;  // scale 1

  RCC->CR |= RCC_CR_HSEON;
  uint32_t timeout = 2000000;
  while (!(RCC->CR & RCC_CR_HSERDY) && --timeout) {
  }
  g_hse_failed = !(RCC->CR & RCC_CR_HSERDY);

  FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;

  // 168 MHz SYSCLK, 48 MHz USB clock (PLLQ). VCO input 1 MHz.
  uint32_t pllm = g_hse_failed ? 16u : 8u;
  RCC->PLLCFGR = (pllm << RCC_PLLCFGR_PLLM_Pos) | (336u << RCC_PLLCFGR_PLLN_Pos) |
                 (0u << RCC_PLLCFGR_PLLP_Pos) | (7u << RCC_PLLCFGR_PLLQ_Pos) |
                 (g_hse_failed ? 0 : RCC_PLLCFGR_PLLSRC_HSE);
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY)) {
  }
  RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2 |
              RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
  }
  RCC->DCKCFGR2 &= ~RCC_DCKCFGR2_CK48MSEL;  // 48 MHz from PLLQ
  SystemCoreClock = 168000000u;
}

void hw_init(void) {
  SCB_EnableICache();
  clock_init();

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SYSCFGEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
  (void)RCC->AHB2ENR;

  // 1 ms tick and free-running 1 MHz TIM2 (APB1 timer clock 84 MHz).
  SysTick_Config(SystemCoreClock / 1000u);
  TIM2->PSC = 83;
  TIM2->ARR = 0xFFFFFFFFu;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->CR1 = TIM_CR1_CEN;

  // LED PC15, IMU INT1 PC3 (input, pulled down).
  GPIOC->BSRR = 1u << 15;
  gpio_mode(GPIOC, 15, 1, 0, 0);
  gpio_mode(GPIOC, 3, 0, 0, 0);
  GPIOC->PUPDR = (GPIOC->PUPDR & ~(3u << 6)) | (2u << 6);

  // Chip selects high: PA4 (gyro 1), PB2 (gyro 2 footprint).
  GPIOA->BSRR = 1u << 4;
  GPIOB->BSRR = 1u << 2;
  gpio_mode(GPIOA, 4, 1, 0, 3);
  gpio_mode(GPIOB, 2, 1, 0, 3);

  // SPI1 on PA5/6/7 AF5, mode 3, 84 MHz / 8 = 10.5 MHz, 8-bit.
  gpio_mode(GPIOA, 5, 2, 5, 3);
  gpio_mode(GPIOA, 6, 2, 5, 3);
  gpio_mode(GPIOA, 7, 2, 5, 3);
  SPI1->CR1 = 0;
  SPI1->CR2 = (7u << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (2u << SPI_CR1_BR_Pos) |
              SPI_CR1_CPOL | SPI_CR1_CPHA;
  SPI1->CR1 |= SPI_CR1_SPE;

  // USB OTG FS on PA11/PA12 AF10.
  gpio_mode(GPIOA, 11, 2, 10, 3);
  gpio_mode(GPIOA, 12, 2, 10, 3);
  NVIC_SetPriority(OTG_FS_IRQn, 2);
}

uint32_t micros(void) { return TIM2->CNT; }

void delay_us(uint32_t us) {
  uint32_t start = micros();
  while ((uint32_t)(micros() - start) < us) {
  }
}

void delay_ms(uint32_t ms) { delay_us(ms * 1000u); }

void led_set(bool on) { GPIOC->BSRR = on ? (1u << (15 + 16)) : (1u << 15); }

void spi_cs(bool active) {
  if (active) {
    GPIOA->BSRR = 1u << (4 + 16);
  } else {
    while (SPI1->SR & SPI_SR_BSY) {
    }
    GPIOA->BSRR = 1u << 4;
  }
}

uint8_t spi_xfer(uint8_t b) {
  while (!(SPI1->SR & SPI_SR_TXE)) {
  }
  *(volatile uint8_t *)&SPI1->DR = b;
  while (!(SPI1->SR & SPI_SR_RXNE)) {
  }
  return *(volatile uint8_t *)&SPI1->DR;
}

bool imu_drdy_pin(void) { return (GPIOC->IDR & (1u << 3)) != 0; }

void hw_uid(uint32_t out[3]) {
  const volatile uint32_t *uid = (const volatile uint32_t *)UID_BASE_ADDR;
  out[0] = uid[0];
  out[1] = uid[1];
  out[2] = uid[2];
}

void hw_reboot(void) { NVIC_SystemReset(); }

void hw_reboot_to_dfu(void) {
  s_boot_magic = DFU_MAGIC;
  __DSB();
  NVIC_SystemReset();
}

// ---- flash config storage ----

static void flash_unlock(void) {
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = 0x45670123u;
    FLASH->KEYR = 0xCDEF89ABu;
  }
}

static bool flash_wait(void) {
  while (FLASH->SR & FLASH_SR_BSY) {
  }
  uint32_t err = FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                              FLASH_SR_PGPERR | FLASH_SR_ERSERR);
  FLASH->SR = err | FLASH_SR_EOP;
  return err == 0;
}

static bool flash_erase_sector(void) {
  flash_unlock();
  flash_wait();
  FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER | (CFG_SECTOR << FLASH_CR_SNB_Pos);
  FLASH->CR |= FLASH_CR_STRT;
  __DSB();
  bool ok = flash_wait();
  FLASH->CR = 0;
  return ok;
}

bool flash_erase_store(void) {
  __disable_irq();
  bool ok = flash_erase_sector();
  FLASH->CR |= FLASH_CR_LOCK;
  __enable_irq();
  return ok;
}

bool flash_store(const void *data, uint32_t len) {
  // Erase takes ~1 s for a 128 KiB sector; USB pauses briefly, host reconnects
  // its stream automatically.
  bool ok = flash_erase_sector();
  if (ok) {
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
    const uint8_t *src = (const uint8_t *)data;
    for (uint32_t off = 0; off < len && ok; off += 4) {
      uint32_t w = 0xFFFFFFFFu;
      memcpy(&w, src + off, (len - off) >= 4 ? 4 : (len - off));
      *(volatile uint32_t *)(CFG_ADDR + off) = w;
      __DSB();
      ok = flash_wait();
    }
    FLASH->CR = 0;
  }
  FLASH->CR |= FLASH_CR_LOCK;
  return ok;
}

const void *flash_load_addr(void) { return (const void *)CFG_ADDR; }
