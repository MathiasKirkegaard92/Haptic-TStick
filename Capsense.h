#include <Wire.h>
#include "Arduino.h"

// Capsense Definitions
#define SENSOR_EN 0x00
#define FSS_EN 0x02
#define SENSITIVITY0 0x08
#define SENSITIVITY1 0x09
#define SENSITIVITY2 0x0A
#define SENSITIVITY3 0x0B
#define DEVICE_ID 0x90    // Should return 0xA05 (returns 2 bytes)
#define FAMILY_ID 0x8F
#define SYSTEM_STATUS 0x8A
#define I2C_ADDR 0x37     // Should return 0x37
#define REFRESH_CTRL 0x52
#define SENSOR_EN 0xFF    // We should set it to 0xFF for 16 sensors
#define BUTTON_STAT 0xAA  // Here we red the status of the sensors (2 bytes)
#define CTRL_CMD 0x86     // To configure the Capsense
#define CTRL_CMD_STATUS 0x88
#define CTRL_CMD_ERROR 0x89
#define BUTTON_LBR 0x1F
#define SPO_CFG 0x4C      //CS15 configuration address
#define GPO_CFG 0x40
#define CALC_CRC 0x94
#define CONFIG_CRC 0x7E

class Capsense {
public:
	Capsense(uint8_t mask1, uint8_t mask2, TwoWire * the_wire);
	uint8_t touch[2];
	uint8_t touchMask[2];
	boolean changed;
	TwoWire * _wire;
	void setup(TwoWire *the_wire);
	uint8_t update();
	uint8_t * readTouch();
};
