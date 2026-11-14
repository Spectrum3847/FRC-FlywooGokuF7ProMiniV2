#include <string.h>

#include "hw.h"
#include "tusb.h"

// pid.codes test VID/PID (0x1209/0x0001) for private/internal use. The host
// library finds devices by the product string, so this can change later.
#define USB_VID 0x1209
#define USB_PID 0x0001

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = (FW_MAJOR << 8) | (FW_MINOR << 4) | FW_PATCH,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

const uint8_t *tud_descriptor_device_cb(void) { return (const uint8_t *)&desc_device; }

enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL };
#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t desc_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
  return desc_config;
}

static const char *const string_desc[] = {
    NULL,           // 0: language (handled below)
    "FRC",          // 1: manufacturer
    "GokuIMU",      // 2: product (host library matches on this)
    NULL,           // 3: serial (MCU unique ID)
    "GokuIMU CDC",  // 4: CDC interface
};

static uint16_t desc_str[33];

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;
  uint8_t count;
  char serial[25];
  const char *str;
  if (index == 0) {
    desc_str[1] = 0x0409;
    count = 1;
  } else {
    if (index == 3) {
      uint32_t uid[3];
      hw_uid(uid);
      static const char hex[] = "0123456789ABCDEF";
      for (int w = 0; w < 3; w++)
        for (int n = 0; n < 8; n++) serial[w * 8 + n] = hex[(uid[w] >> (28 - n * 4)) & 0xF];
      serial[24] = 0;
      str = serial;
    } else if (index < sizeof(string_desc) / sizeof(string_desc[0])) {
      str = string_desc[index];
    } else {
      return NULL;
    }
    count = (uint8_t)strlen(str);
    if (count > 32) count = 32;
    for (uint8_t i = 0; i < count; i++) desc_str[1 + i] = (uint8_t)str[i];
  }
  desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * count + 2));
  return desc_str;
}

uint32_t tusb_time_millis_api(void) { return g_millis; }
