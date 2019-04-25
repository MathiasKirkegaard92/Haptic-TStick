/*!
 * @file DRV2605.cpp
 *
 * @mainpage Adafruit DRV2605L Haptic Driver
 *
 * @section intro_sec Introduction
 *
 * This is a library for the Adafruit DRV2605L Haptic Driver ----> http://www.adafruit.com/products/2305
 *
 * Check out the links above for our tutorials and wiring diagrams.
 *
 * This motor/haptic driver uses I2C to communicate.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text above must be included in any redistribution.
 *
 */
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "DRV2605.h"
#include <Wire.h>


/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
  @brief  Instantiates a new DRV2605 class. I2C, no address adjustments or pins
*/
/**************************************************************************/
DRV2605::DRV2605() {
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
  @brief Setup HW using a specified Wire
  @param theWire Pointer to a TwoWire object, defaults to &Wire
  @return Return value from init()
*/
/**************************************************************************/
boolean DRV2605::begin(TwoWire *theWire,uint8_t sdaPin, uint8_t sclPin) {
  _wire = theWire;
  return init(sdaPin,sclPin);
}

/**************************************************************************/
/*!
  @brief  Setup the HW
  @return Always true
*/
/**************************************************************************/
boolean DRV2605::init(uint8_t sdaPin, uint8_t sclPin) {
  _wire->begin(sdaPin,sclPin,400000);
  uint8_t id = readRegister8(DRV2605_REG_STATUS);
  //Serial.print("Status 0x"); Serial.println(id, HEX);

  writeRegister8(DRV2605_REG_MODE, 0x00); // out of standby

  writeRegister8(DRV2605_REG_RTPIN, 0x00); // no real-time-playback

  writeRegister8(DRV2605_REG_WAVESEQ1, 1); // strong click
  writeRegister8(DRV2605_REG_WAVESEQ2, 0); // end sequence

  writeRegister8(DRV2605_REG_OVERDRIVE, 0); // no overdrive

  writeRegister8(DRV2605_REG_SUSTAINPOS, 0);
  writeRegister8(DRV2605_REG_SUSTAINNEG, 0);
  writeRegister8(DRV2605_REG_BREAK, 0);
  writeRegister8(DRV2605_REG_AUDIOMAX, 0x64);

  // ERM open loop

  // turn off N_ERM_LRA
  writeRegister8(DRV2605_REG_FEEDBACK, readRegister8(DRV2605_REG_FEEDBACK) & 0x7F);
  // turn on ERM_OPEN_LOOP
  writeRegister8(DRV2605_REG_CONTROL3, readRegister8(DRV2605_REG_CONTROL3) | 0x20);

  return true;
}

/**************************************************************************/
/*!
  @brief Select the haptic waveform to use.
  @param slot The waveform slot to set, from 0 to 7
  @param w The waveform sequence value, refers to an index in the ROM library.

    Playback starts at slot 0 and continues through to slot 7, stopping if it encounters
    a value of 0. A list of available waveforms can be found in section 11.2
    of the datasheet: http://www.adafruit.com/datasheets/DRV2605.pdf
*/
/**************************************************************************/
void DRV2605::setWaveform(uint8_t slot, uint8_t w) {
  writeRegister8(DRV2605_REG_WAVESEQ1+slot, w);
}

/**************************************************************************/
/*!
  @brief Select the waveform library to use.
  @param lib Library to use, 0 = Empty, 1-5 are ERM, 6 is LRA.

    See section 7.6.4 in the datasheet for more details: http://www.adafruit.com/datasheets/DRV2605.pdf
*/
/**************************************************************************/
void DRV2605::selectLibrary(uint8_t lib) {
  writeRegister8(DRV2605_REG_LIBRARY, lib);
}

/**************************************************************************/
/*!
  @brief Start playback of the waveforms (start moving!).
*/
/**************************************************************************/
void DRV2605::go() {
  writeRegister8(DRV2605_REG_GO, 1);
}

uint8_t DRV2605::goStatus(){
	return readRegister8(DRV2605_REG_GO);
}

/**************************************************************************/
/*!
  @brief Stop playback.
*/
/**************************************************************************/
void DRV2605::stop() {
  writeRegister8(DRV2605_REG_GO, 0);
}

/**************************************************************************/
/*!
  @brief Set the device mode.
  @param mode Mode value, see datasheet section 7.6.2: http://www.adafruit.com/datasheets/DRV2605.pdf

    0: Internal trigger, call go() to start playback\n
    1: External trigger, rising edge on IN pin starts playback\n
    2: External trigger, playback follows the state of IN pin\n
    3: PWM/analog input\n
    4: Audio\n
    5: Real-time playback\n
    6: Diagnostics\n
    7: Auto calibration
*/
/**************************************************************************/
void DRV2605::setMode(uint8_t mode) {
  writeRegister8(DRV2605_REG_MODE, mode);
}

/**************************************************************************/
/*!
  @brief Set the realtime value when in RTP mode, used to directly drive the haptic motor.
  @param rtp 8-bit drive value.
*/
/**************************************************************************/
void DRV2605::setRealtimeValue(uint8_t rtp) {
  writeRegister8(DRV2605_REG_RTPIN, rtp);
}


/**************************************************************************/
/*!
  @brief Read an 8-bit register.
  @param reg The register to read.
  @return 8-bit value of the register.
*/
/**************************************************************************/
uint8_t DRV2605::readRegister8(uint8_t reg) {
  uint8_t x;

  // use i2c
  _wire->beginTransmission(DRV2605_ADDR);
  _wire->write((byte)reg);
  _wire->endTransmission();
  _wire->requestFrom((byte)DRV2605_ADDR, (byte)1);
  x = _wire->read();

  //  Serial.print("$"); Serial.print(reg, HEX);
  //  Serial.print(": 0x"); Serial.println(x, HEX);

  return x;
}

/**************************************************************************/
/*!
  @brief Write an 8-bit register.
  @param reg The register to write.
  @param val The value to write.
*/
/**************************************************************************/
void DRV2605::writeRegister8(uint8_t reg, uint8_t val) {
  // use i2c
  _wire->beginTransmission(DRV2605_ADDR);
  _wire->write((byte)reg);
  _wire->write((byte)val);
  _wire->endTransmission();
}

/**************************************************************************/
/*!
  @brief Use ERM (Eccentric Rotating Mass) mode.
*/
/**************************************************************************/
void DRV2605::useERM () {
  writeRegister8(DRV2605_REG_FEEDBACK, readRegister8(DRV2605_REG_FEEDBACK) & 0x7F);
}

/**************************************************************************/
/*!
  @brief Use LRA (Linear Resonance Actuator) mode.
*/
/**************************************************************************/
void DRV2605::useLRA (float v_rms, float v_peak, float f_res) {
  writeRegister8(DRV2605_REG_FEEDBACK, readRegister8(DRV2605_REG_FEEDBACK) | 0x88); // Use LRA 0x80, set open loop gain to 2 , 0x08

  // Set rated voltage
  float V_BINARY = (v_rms * sqrt(1-(4*0.0003+0.0003)*f_res)*255.0/5.3);
  float V_BINARY_CLAMP = (v_peak * sqrt(1-(4*0.0003+0.0003)*f_res)*255.0/5.3);
  writeRegister8(DRV2605_REG_RATEDV, uint8_t(V_BINARY));
  writeRegister8(DRV2605_REG_CLAMPV, uint8_t(V_BINARY_CLAMP));
}

void DRV2605::autoCalibrate(){
	  writeRegister8(DRV2605_REG_MODE, 0x00); // out of standby
	  // Set control registers. defualt for now
	  // Control 1 - set drive time to 19
	  writeRegister8(DRV2605_REG_CONTROL1, 0x13);
	  setMode(7); // set mode to autocalibration

	  // Autocalibration parameters - set calibration time to maximum
	  writeRegister8(DRV2605_REG_CONTROL4, readRegister8(DRV2605_REG_CONTROL4) | 0x30);
	  // Start calibration 
	  go();
	  while(goStatus() == 1){
	  	delay(500);
	  	Serial.println("calibrating");
	  }
	  if(readRegister8(0xA8) == 0){
	  	Serial.println("autocalibration succeded");	
	  } else Serial.println("autocalibration failed");
	  Serial.println("autocalibration results:");
	  Serial.print("0x18 "); Serial.println(readRegister8(0x18));
	  Serial.print("0x19"); Serial.println(readRegister8(0x19));
	  Serial.print("0x1A"); Serial.println(readRegister8(0x1A));
}
  void DRV2605::setupLRA(TwoWire* the_wire, int sdaPin, int sclPin){
    begin(the_wire,sdaPin,sclPin);
    useLRA(1.8,2, 235.0);
    autoCalibrate();

  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
    // setMode(DRV2605_MODE_INTTRIG); 
    setMode(DRV2605_MODE_PWMANALOG);
  };
