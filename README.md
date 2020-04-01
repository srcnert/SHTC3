# SHTC3 Library

Initialize:

    /* SHTC3 */
    // Initialize I2C Bus
    i2c_clear_bus(I2C_SCL, I2C_SDA);
    i2c_init();
    
    // initialize sensor module with the i2c address 0x70
    etError  error;       // error code
    uint16_t id;          // sensor ID
    uint16_t temperature; // temperature
    uint16_t humidity;    // relative humidity

    shtc3_dev_t shtc3_dev;
    shtc3_dev.dev_id = 0x70;
    shtc3_dev.i2c_write = &my_i2c_write;
    shtc3_dev.i2c_read = &my_i2c_read;
    shtc3_dev.delay_us = &nrfx_coredep_delay_us;
    SHTC3_Init(shtc3_dev);
    
    // wake up the sensor from sleep mode
    error = SHTC3_Wakeup();
    if(error != NO_ERROR)
    {
      NVIC_SystemReset();
    }
    
    // demonstration of SoftReset command
    error = SHTC3_SoftReset();
    if(error != NO_ERROR)
    {
      NVIC_SystemReset();
    }

    // wait for sensor to reset
    delay_ms(100);

    // demonstration of GetId command
    error = SHTC3_GetId(&id);
    if(error == NO_ERROR)
    {
    }
    else
    {
      NVIC_SystemReset();
    }

    // ID Check
    if((id & 0x083F) == 0x0807)   // Checking the form of the ID
    {                             // Bits 11 and 5-0 must match
	// SHTC3 CORRECT ID
    }

    // Start Measurement
    error = SHTC3_GetTempAndHumiPollingRaw(&temperature, &humidity);
    if(error == NO_ERROR)
    {
      printf("SHTC3 TEMP: %d",temperature);
      printf("SHTC3 HUM: %d", humidity);
    }
    else
    {
      printf("STC3 COMM ERROR: %d", error);
      NVIC_SystemReset();
    }

    // activate the sleep mode of the sensor to save energy
    error = SHTC3_Sleep();
    if(error != NO_ERROR)
      NVIC_SystemReset();
    /* End of SHTC3 */

Usage:

    /* Start Measurement of SHTC3 */
    etError  error=0;        // error code
    uint16_t temperature=0;  // temperature
    uint16_t humidity=0;     // relative humidity

    // wake up the sensor from sleep mode
    error = SHTC3_Wakeup();
    if(error != NO_ERROR)
      NVIC_SystemReset();

    error = SHTC3_GetTempAndHumiPollingRaw(&temperature, &humidity);
    if(error != NO_ERROR)
      NVIC_SystemReset();
    
    // activate the sleep mode of the sensor to save energy
    error = SHTC3_Sleep();
    if(error != NO_ERROR)
      NVIC_SystemReset();

