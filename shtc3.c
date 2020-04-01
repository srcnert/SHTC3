#include "shtc3.h"

typedef enum{
  READ_ID            = 0xEFC8, // command: read ID register
  SOFT_RESET         = 0x805D, // soft reset
  SLEEP              = 0xB098, // sleep
  WAKEUP             = 0x3517, // wakeup
  MEAS_T_RH_POLLING  = 0x7866, // meas. read T first, clock stretching disabled (normal mode)
  MEAS_T_RH_CLOCKSTR = 0x7CA2, // meas. read T first, clock stretching enabled (normal mode)
  MEAS_RH_T_POLLING  = 0x58E0, // meas. read RH first, clock stretching disabled (normal mode)
  MEAS_RH_T_CLOCKSTR = 0x5C24, // meas. read RH first, clock stretching enabled (normal mode)
//  MEAS_T_RH_POLLING_LP  = 0x609C, // meas. read T first, clock stretching disabled (low power mode)
//  MEAS_T_RH_CLOCKSTR_LP = 0x6458, // meas. read T first, clock stretching enabled (low power mode)
//  MEAS_RH_T_POLLING_LP  = 0x401A, // meas. read RH first, clock stretching disabled (low power mode)
//  MEAS_RH_T_CLOCKSTR_LP = 0x44DE  // meas. read RH first, clock stretching enabled (low power mode)
}etCommands;

static etError SHTC3_Read2BytesAndCrc(uint16_t *data);
static etError SHTC3_Read4BytesAndCrc(uint16_t *data1, uint16_t *data2);
static etError SHTC3_WriteCommand(etCommands cmd);
static etError SHTC3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes,
                              uint8_t checksum);
static float SHTC3_CalcTemperature(uint16_t rawValue);
static float SHTC3_CalcHumidity(uint16_t rawValue);


//------------------------------------------------------------------------------
shtc3_dev_t _shtc3_dev;
void SHTC3_Init(shtc3_dev_t shtc3_dev){
  _shtc3_dev = shtc3_dev;
}

//------------------------------------------------------------------------------
etError SHTC3_GetTempAndHumi(float *temp, float *humi){
  etError  error;        // error code
  uint16_t rawValueTemp; // temperature raw value from sensor
  uint16_t rawValueHumi; // humidity raw value from sensor

  // measure, read temperature first, clock streching enabled
  error = SHTC3_WriteCommand(MEAS_T_RH_CLOCKSTR);

  // if no error, read temperature and humidity raw values
  if(error == NO_ERROR) {
    error |= SHTC3_Read4BytesAndCrc(&rawValueTemp, &rawValueHumi);
  }

  // if no error, calculate temperature in C and humidity in %RH
  if(error == NO_ERROR) {
    *temp = SHTC3_CalcTemperature(rawValueTemp);
    *humi = SHTC3_CalcHumidity(rawValueHumi);
  }

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_GetTempAndHumiRaw(uint16_t *temp, uint16_t *humi){
  etError  error;        // error code

  // measure, read temperature first, clock streching enabled
  error = SHTC3_WriteCommand(MEAS_T_RH_CLOCKSTR);

  // if no error, read temperature and humidity raw values
  if(error == NO_ERROR) {
    error |= SHTC3_Read4BytesAndCrc(temp, humi);
  }

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_GetTempAndHumiPolling(float *temp, float *humi){
  etError  error;           // error code
  uint8_t  maxPolling = 20; // max. retries to read the measurement (polling)
  uint16_t rawValueTemp;    // temperature raw value from sensor
  uint16_t rawValueHumi;    // humidity raw value from sensor

  // measure, read temperature first, clock streching disabled (polling)
  error = SHTC3_WriteCommand(MEAS_T_RH_POLLING);

  // if no error, ...
  if(error == NO_ERROR) {
    // poll every 1ms for measurement ready
    while(maxPolling--) {
      // check if the measurement has finished
      error = SHTC3_Read4BytesAndCrc(&rawValueTemp, &rawValueHumi);

      // if measurement has finished -> exit loop
      if(error == NO_ERROR) break;

      // delay 1ms
      _shtc3_dev.delay_us(1000);
    }
  }

  // if no error, calculate temperature in C and humidity in %RH
  if(error == NO_ERROR) {
    *temp = SHTC3_CalcTemperature(rawValueTemp);
    *humi = SHTC3_CalcHumidity(rawValueHumi);
  }

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_GetTempAndHumiPollingRaw(uint16_t *temp, uint16_t *humi){
  etError  error;           // error code
  uint8_t  maxPolling = 20; // max. retries to read the measurement (polling)

  // measure, read temperature first, clock streching disabled (polling)
  error = SHTC3_WriteCommand(MEAS_T_RH_POLLING);

  // if no error, ...
  if(error == NO_ERROR) {
    // poll every 1ms for measurement ready
    while(maxPolling--) {
      // check if the measurement has finished
      error = SHTC3_Read4BytesAndCrc(temp, humi);

      // if measurement has finished -> exit loop
      if(error == NO_ERROR) break;

      // delay 1ms
      _shtc3_dev.delay_us(1000);
    }
  }

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_GetId(uint16_t *id){
  etError error; // error code

  // write ID read command
  error = SHTC3_WriteCommand(READ_ID);

  // if no error, read ID
  if(error == NO_ERROR) {
    error = SHTC3_Read2BytesAndCrc(id);
  }

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_Sleep(void) {
  etError error = SHTC3_WriteCommand(SLEEP);

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_Wakeup(void) {
  etError error = SHTC3_WriteCommand(WAKEUP);

  _shtc3_dev.delay_us(100); // wait 100 us

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_SoftReset(void){
  etError error; // error code

  // write reset command
  error = SHTC3_WriteCommand(SOFT_RESET);

  return error;
}

//------------------------------------------------------------------------------
static etError SHTC3_WriteCommand(etCommands cmd){
  etError error; // error code
  uint8_t wBuffer[2];
  
  wBuffer[0] = cmd >> 8;
  wBuffer[1] = cmd & 0xFF;

  error = _shtc3_dev.i2c_write(_shtc3_dev.dev_id, wBuffer, 2);
  if(error != NO_ERROR)
    return I2C_ERROR;

  return error;
}

//------------------------------------------------------------------------------
static etError SHTC3_Read2BytesAndCrc(uint16_t *data){
  etError error;      // error code
  uint8_t rBuffer[3]; // read data array
  uint8_t checksum;   // checksum byte

  // read two data bytes and one checksum byte
  error = _shtc3_dev.i2c_read(_shtc3_dev.dev_id, rBuffer, 3);
  if(error != NO_ERROR)
    return I2C_ERROR;

  checksum = rBuffer[2];
  
  // verify checksum
  error = SHTC3_CheckCrc(&rBuffer[0], 2, checksum);

  // combine the two bytes to a 16-bit value
  *data = (rBuffer[0] << 8) | rBuffer[1];

  return error;
}

//------------------------------------------------------------------------------
static etError SHTC3_Read4BytesAndCrc(uint16_t *data1, uint16_t *data2){
  etError error;      // error code
  uint8_t rBuffer[6]; // read data array
  uint8_t checksum;   // checksum byte

  // read two data bytes and one checksum byte
  error = _shtc3_dev.i2c_read(_shtc3_dev.dev_id, rBuffer, 6);
  if(error != NO_ERROR)
    return I2C_ERROR;

  checksum = rBuffer[2];
  
  // verify checksum
  error = SHTC3_CheckCrc(&rBuffer[0], 2, checksum);
  if(error != NO_ERROR)
    return error;

  checksum = rBuffer[5];
  
  // verify checksum
  error = SHTC3_CheckCrc(&rBuffer[3], 2, checksum);
  if(error != NO_ERROR)
    return error;

  // combine the two bytes to a 16-bit value
  *data1 = (rBuffer[0] << 8) | rBuffer[1];
  *data2 = (rBuffer[3] << 8) | rBuffer[4];

  return error;
}

//------------------------------------------------------------------------------
static etError SHTC3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes,
                              uint8_t checksum){
  uint8_t crc = 0xFF; // calculated checksum

  // calculates 8-Bit checksum with given polynomial
  for(uint8_t byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
    crc ^= (data[byteCtr]);
    for(uint8_t bit = 8; bit > 0; --bit) {
      if(crc & 0x80) {
        crc = (crc << 1) ^ CRC_POLYNOMIAL;
      } else {
        crc = (crc << 1);
      }
    }
  }

  // verify checksum
  if(crc != checksum) {
    return CHECKSUM_ERROR;
  } else {
    return NO_ERROR;
  }
}

//------------------------------------------------------------------------------
static float SHTC3_CalcTemperature(uint16_t rawValue){
  // calculate temperature [C]
  // T = -45 + 175 * rawValue / 2^16
  return 175 * (float)rawValue / 65536.0f - 45.0f;
}

//------------------------------------------------------------------------------
static float SHTC3_CalcHumidity(uint16_t rawValue){
  // calculate relative humidity [%RH]
  // RH = rawValue / 2^16 * 100
  return 100 * (float)rawValue / 65536.0f;
}
