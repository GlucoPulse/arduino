#ifndef ICON_H
#define ICON_H

#include <pgmspace.h>
#include <stdint.h>

// const uint8_t battery_bitmap[] PROGMEM = {
// 	0x7F, 0x81, 0x81, 0x81, 0x7F, 0x00, 0x00, 0x00, // top part (empty battery, no charge)
// 	0x7F, 0x81, 0x81, 0xA1, 0x7F, 0x00, 0x00, 0x00, // top part (half-charged battery)
// 	0x7F, 0x81, 0x91, 0xA1, 0x7F, 0x00, 0x00, 0x00	// top part (full battery) };
// };

// const uint8_t wifi_icon[] PROGMEM = {
// 	0x1C, 0x36, 0x63, 0x63, 0x36, 0x1C, 0x00, 0x00 // Simple WiFi icon
// };

// Battery icons (16x8 pixels)
// Empty battery outline
static const unsigned char battery_empty_bmp[] = {
  0x03, 0xFF, 0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 
  0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 0x03, 0xFF
};

// Battery levels (16x8 pixels)
static const unsigned char battery_25_bmp[] = {
  0x03, 0xFF, 0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 
  0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 0x03, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char battery_50_bmp[] = {
  0x03, 0xFF, 0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 
  0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 0x03, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char battery_75_bmp[] = {
  0x03, 0xFF, 0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 
  0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 0x03, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char battery_100_bmp[] = {
  0x03, 0xFF, 0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 
  0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 0x03, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char battery_charging_bmp[] = {
  0x03, 0xFF, 0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 
  0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 0x03, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};




const uint8_t battery_bitmap[] PROGMEM = {
	0b00111111, 0b11100000,
	0b00100000, 0b00100000,
	0b00101111, 0b10100000,
	0b00101111, 0b10100000,
	0b00101111, 0b10100000,
	0b00101111, 0b10100000,
	0b00100000, 0b00100000,
	0b00111111, 0b11100000};

// 12x8 WiFi icon (3 bars)
const uint8_t wifi_icon[] PROGMEM = {
	0b00011110, 0b00000000,
	0b00100001, 0b00000000,
	0b01011011, 0b10000000,
	0b10000000, 0b01000000,
	0b10010101, 0b01000000,
	0b01000000, 0b10000000,
	0b00100001, 0b00000000,
	0b00011110, 0b00000000};

  // Static battery icons (16x8 pixels)
static const unsigned char battery_full_bmp[] = {
  0x03, 0xFF, 0xFD, 0x05, 0x05, 0x05, 0x05, 0x05, 
  0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0xFD, 0xFF
};

#endif