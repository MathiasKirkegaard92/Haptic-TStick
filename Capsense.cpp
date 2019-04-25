#include "Capsense.h"
Capsense::Capsense(uint8_t mask1 = 255, uint8_t mask2 = 255, TwoWire * the_wire = &Wire): touch {0,0}, touchMask {mask1,mask2}, changed (true), _wire(the_wire){
}
void Capsense::setup(TwoWire *the_wire){
  if (0) {
  ///////
    byte infoboard = 0;
    the_wire->beginTransmission(I2C_ADDR);
    the_wire->write(SYSTEM_STATUS);
    the_wire->endTransmission();
  
    // This should be 0 since a configuration other than the factory is loaded.
    the_wire->requestFrom(I2C_ADDR,1);
    Serial.print("the_wire->available First: ");
    Serial.println(the_wire->available());
    infoboard = the_wire->read();
    Serial.println("This should be 0");
    Serial.print("SYSTEM_STATUS: ");
    Serial.println(infoboard);
    the_wire->endTransmission();
  
  ///////
    the_wire->beginTransmission(I2C_ADDR);
    the_wire->write(FAMILY_ID);
    the_wire->endTransmission();
  
    the_wire->requestFrom(I2C_ADDR,1);
    Serial.print("the_wire->available SECOND: ");
    Serial.println(the_wire->available());
    infoboard = the_wire->read();
    Serial.println("This should be 154");
    Serial.print("FAMILY_ID: ");
    Serial.println(infoboard);
    the_wire->endTransmission();
  
  
  //  the_wire->beginTransmission(I2C_ADDR);
  //  the_wire->write(DEVICE_ID);
  //  the_wire->endTransmission();
  //  //delay(100);
  //
  //  the_wire->requestFrom(I2C_ADDR,2);
  //  Serial.print("the_wire->available THIRD: ");
  //  Serial.println(the_wire->available());
  //  while (the_wire->available()) { 
  //    byte c = the_wire->read(); 
  //    Serial.println(c); 
  //  }  
    //delay(100);
  //  the_wire->beginTransmission(I2C_ADDR);
  //  the_wire->write(REFRESH_CTRL);
  //  the_wire->endTransmission();
  //  //delay(100);
  //
  //  the_wire->requestFrom(I2C_ADDR,1);
  //  Serial.print("the_wire->available sixth: ");
  //  Serial.println(the_wire->available());
  //  infoboard = the_wire->read();
  //  Serial.print("REFRESH_CTRL VALUE: ");
  //  Serial.println(infoboard);
  //  the_wire->endTransmission();
    //delay(100);
  }
  
  //**********************************  
  // Capsense pins configuration 
  //
  // Array to enable each of the 16 capsense sensors 
  byte sensors[2] = {0xFFu,0xFFu};

  // Originally the come as 0x00, 0xFF
  the_wire->beginTransmission(I2C_ADDR);
  the_wire->write(SENSOR_EN);
  the_wire->write(sensors,2);
  the_wire->endTransmission();

  // Not actually necessary I think.
  the_wire->beginTransmission(I2C_ADDR);
  the_wire->write(FSS_EN);
  //the_wire->write(sensors,2);
  the_wire->write(0x00);
  the_wire->write(0x00);
  the_wire->endTransmission();

//  // Sensitivity
 the_wire->beginTransmission(I2C_ADDR);
 the_wire->write(SENSITIVITY0);
 the_wire->write(B10101010);
 the_wire->endTransmission();

 the_wire->beginTransmission(I2C_ADDR);
 the_wire->write(SENSITIVITY1);
 the_wire->write(B10101010);
 the_wire->endTransmission();

 the_wire->beginTransmission(I2C_ADDR);
 the_wire->write(SENSITIVITY2);
 the_wire->write(B10101010);
 the_wire->endTransmission();

 the_wire->beginTransmission(I2C_ADDR);
 the_wire->write(SENSITIVITY3);
 the_wire->write(B10101010);
 the_wire->endTransmission();

  // Special Purpose Output pin configuration
  the_wire->beginTransmission(I2C_ADDR);
  the_wire->write(SPO_CFG);
  the_wire->write(B00010101);
  the_wire->endTransmission();
//
//  the_wire->beginTransmission(I2C_ADDR);
//  the_wire->write(SPO_CFG);
//  the_wire->endTransmission();
//  //delay(100);
//
//  the_wire->requestFrom(I2C_ADDR,1);
//  Serial.print("the_wire->available SPO_CFG: ");
//  Serial.println(the_wire->available());
//  infoboard = the_wire->read();
//  Serial.print("SPO_CFG: ");
//  Serial.println(infoboard,BIN);
//  the_wire->endTransmission();

  // Let's read again the SENSOR_EN just to confirm that nothing has been written yet. To be Deleted
  the_wire->beginTransmission(I2C_ADDR);
  the_wire->write(SENSOR_EN);
  the_wire->endTransmission();

  the_wire->requestFrom(I2C_ADDR,2);
  Serial.print("the_wire->available FOURTH: ");
  Serial.println(the_wire->available());
  while (the_wire->available()) { // slave may send less than requested
    byte c = the_wire->read(); // receive a byte as character
    Serial.println(c);         // print the character
  }  

/*
 *  WRITING SETTINGS IN CAPSENSE
    - Write via I2C all the commands necessary to configure Capsense. I think the idea here is to have a conditional configuration option in the firmware and only enter this step if newer configuration needs to be loaded. All values are stored in internal flash memory so this could be bypassed in final firmware. 
    - Write ‘3’ to CTRL_CMD in order to generate CRC that is written automatically by the chip to CALC_CRC
    - After 220ms or more read CRC from CALC_CRC
    - Write it to CONFIG_CRC
    - Write ‘2’ to CTRL_CMD to compare current CRC and stored one in CALC_CRC (they are the same) and write in internal flash.
    - Wait 220ms or more for command to finish. 

 */

  
  // Send 0x03 to calculate de CRC from the changed configuration.
  the_wire->beginTransmission(I2C_ADDR);
  the_wire->write(CTRL_CMD);
  the_wire->write(0x03);
  the_wire->endTransmission();

  // This delay is important
  delay(300);

  // Read CRC calculated from the 0x03 command sent
  byte crc[2] = {0, 0}; 
  int i = 0;
  the_wire->beginTransmission(I2C_ADDR);
  the_wire->write(CALC_CRC);
  the_wire->endTransmission();

  the_wire->requestFrom(I2C_ADDR,2);
  while (the_wire->available()) { // slave may send less than requested
    byte c = the_wire->read();
    Serial.println(c);         // print the character
    crc[i] = c; // receive a byte as character
    i++;
  }    
  the_wire->endTransmission();

  // Write CRC to CONFIG_CRC
  the_wire->beginTransmission(I2C_ADDR);
  the_wire->write(CONFIG_CRC);
  the_wire->write(crc,2);
  the_wire->endTransmission();

  // Send 0x02 to calculate de CRC from the changed configuration.
  the_wire->beginTransmission(I2C_ADDR);
  the_wire->write(CTRL_CMD);
  the_wire->write(0x02);
  the_wire->endTransmission();  
  delay(300);

  int result = 2;

  while (result != 0){
    the_wire->beginTransmission(I2C_ADDR);
    the_wire->write(CTRL_CMD_STATUS);
    the_wire->endTransmission();

    the_wire->requestFrom(I2C_ADDR,1);
    result = the_wire->read();
    Serial.print("result CTRL_CMD_STATUS 1: ");
    Serial.println(result);
    if (result == 1) {
      the_wire->beginTransmission(I2C_ADDR);
      the_wire->write(CTRL_CMD_ERROR);
      the_wire->endTransmission();

      the_wire->requestFrom(I2C_ADDR,1);
      result = the_wire->read();
      Serial.print("result CTRL_CMD_ERROR 1: ");
      Serial.println(result);
      the_wire->endTransmission();
    }
    the_wire->endTransmission();
  }

  the_wire->beginTransmission(I2C_ADDR);
  the_wire->write(CTRL_CMD);
  the_wire->write(0xFF);
  the_wire->endTransmission();
  delay(300);

  result = 2;

  while (result != 0){
    the_wire->beginTransmission(I2C_ADDR);
    the_wire->write(CTRL_CMD_STATUS);
    the_wire->endTransmission();

    the_wire->requestFrom(I2C_ADDR,1);
    result = the_wire->read();
    Serial.print("result CTRL_CMD_STATUS 2: ");
    Serial.println(result);
    if (result == 1) {
      the_wire->beginTransmission(I2C_ADDR);
      the_wire->write(CTRL_CMD_ERROR);
      the_wire->endTransmission();

      the_wire->requestFrom(I2C_ADDR,1);
      result = the_wire->read();
      Serial.print("result CTRL_CMD_ERROR 2: ");
      Serial.println(result);
      the_wire->endTransmission();
    }
    the_wire->endTransmission();
  }  
  delay(500);  
}

uint8_t Capsense::update() {
  changed = 0;
  uint8_t temp[2] = {0, 0}; int i = 0;
  _wire->beginTransmission(I2C_ADDR);
  _wire->write(BUTTON_STAT);
  _wire->endTransmission();

  _wire->requestFrom(I2C_ADDR, 2);
  while (_wire->available()) { // slave may send less than requested
    //    byte c = Wire.read();
    //    temp[i] = c; // receive a byte as character
    temp[i] = _wire->read();
    i++;
  }
  _wire->endTransmission();

  for (int t = 0; t < 2; t++) {
    if (temp[t] != touch[t]) {
      changed = 1;
      touch[t] = temp[t];
    }
  }
  return changed;
}

uint8_t * Capsense::readTouch(){
  changed = 0;
  static uint8_t tmp[2];
  tmp[0] = (uint8_t)touch[0] & touchMask[1]; tmp[1] = (uint8_t)touch[0] & touchMask[1];
  return (uint8_t*) touch;
}


