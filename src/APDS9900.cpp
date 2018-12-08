/**
 * @file    APDS-9900.cpp
 * @brief   Library for the SparkFun APDS-9900 breakout board
 * @author  Shawn Hymel (SparkFun Electronics)
 *
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 *
 * This library interfaces the Avago APDS-9900 to Arduino over I2C. The library
 * relies on the Arduino Wire (I2C) library. to use the library, instantiate an
 * APDS9900 object, call init(), and call the appropriate functions.
 *
 */

 #include <Arduino.h>
 #include <Wire.h>

 #include "APDS9900.h"

/**
 * @brief Constructor - Instantiates APDS9900 object
 */
APDS9900::APDS9900()
{

}

/**
 * @brief Destructor
 */
APDS9900::~APDS9900()
{

}

/**
 * @brief Configures I2C communications and initializes registers to defaults
 *
 * @return True if initialized successfully. False otherwise.
 */
bool APDS9900::init()
{
    uint8_t id;

    /* Initialize I2C */
    // Wire.begin();

    /* Read ID register and check against known values for APDS-9900 */
    if( !wireReadDataByte(APDS9900_ID, id) ) {
      Serial.println("wire read failed.");
        return false;
    }
    if( !(id == APDS9900_ID_1 || id == APDS9900_ID_2) ) {
        Serial.print("ID unknown: "); Serial.println(id);
        return false;
    }

    /* Set default values for ambient light and proximity registers */
    if( !wireWriteDataByte(APDS9900_ATIME, DEFAULT_ATIME) ) {
        Serial.println("Setting ATIME failed.");
        return false;
    }
    if( !wireWriteDataByte(APDS9900_PTIME, DEFAULT_PTIME) ) {
      Serial.println("Setting PTIME failed.");
        return false;
    }
    if( !wireWriteDataByte(APDS9900_WTIME, DEFAULT_WTIME) ) {
        Serial.println("Setting WTIME failed.");
        return false;
    }
    if( !wireWriteDataByte(APDS9900_PPCOUNT, DEFAULT_PPCOUNT) ) {
        Serial.println("Setting PPCOUNT failed.");
        return false;
    }

    if( !wireWriteDataByte(APDS9900_CONTROL, APDS9900_PDRIVE | APDS9900_PDIODE | APDS9900_PGAIN | APDS9900_AGAIN) ) {
      Serial.println("Setting control register failed.");
        return false;
    }
    if( !wireWriteDataByte(APDS9900_ENABLE, APDS9900_WEN | APDS9900_PEN | APDS9900_AEN | APDS9900_PON) ) {
      Serial.println("Setting ENs failed.");
        return false;
    }

    /* extend the minimum viable driver here:
    if( !setLEDDrive(DEFAULT_LDRIVE) ) {
        Serial.println("Setting DEFAULT_LDRIVE failed.");
        return false;
    }
    if( !setProximityGain(DEFAULT_PGAIN) ) {
        Serial.println("Setting DEFAULT_PGAIN failed.");
        return false;
    }
    if( !setAmbientLightGain(DEFAULT_AGAIN) ) {
        Serial.println("Setting DEFAULT_AGAIN failed.");
        return false;
    }
    if( !setProxIntLowThresh(DEFAULT_PILT) ) {
        Serial.println("Setting DEFAULT_PILT failed.");
        return false;
    }
    if( !setProxIntHighThresh(DEFAULT_PIHT) ) {
        Serial.println("Setting DEFAULT_PIHT failed.");
        return false;
    }
    if( !setLightIntLowThreshold(DEFAULT_AILT) ) {
        Serial.println("Setting DEFAULT_AILT failed.");
        return false;
    }
    if( !setLightIntHighThreshold(DEFAULT_AIHT) ) {
        Serial.println("Setting DEFAULT_AIHT failed.");
        return false;
    }
    */

    return true;
}

/*******************************************************************************
 * Public methods for controlling the APDS-9900
 ******************************************************************************/

/**
 * @brief Reads and returns the contents of the ENABLE register
 *
 * @return Contents of the ENABLE register. 0xFF if error.
 */
uint8_t APDS9900::getMode()
{
    uint8_t enable_value;

    /* Read current ENABLE register */
    if( !wireReadDataByte(APDS9900_ENABLE, enable_value) ) {
        return ERROR;
    }

    return enable_value;
}

/**
 * @brief Enables or disables a feature in the APDS-9900
 *
 * @param[in] mode which feature to enable
 * @param[in] enable ON (1) or OFF (0)
 * @return True if operation success. False otherwise.
 */
bool APDS9900::setMode(apds_mode_t mode, uint8_t enable)
{
    uint8_t reg_val;

    /* Read current ENABLE register */
    reg_val = getMode();
    if( reg_val == ERROR ) {
        return false;
    }

    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if( mode >= 0 && mode <= 5 ) {
        if (enable) {
            reg_val |= (1 << mode);
        } else {
            reg_val &= ~(1 << mode);
        }
    } else if( mode == ALL ) {
        if (enable) {
            reg_val = 0x7F;
        } else {
            reg_val = 0x00;
        }
    }

    /* Write value back to ENABLE register */
    if( !wireWriteDataByte(APDS9900_ENABLE, reg_val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Starts the light (Ambient) sensor on the APDS-9900
 *
 * @param[in] interrupts true to enable hardware interrupt on high or low light
 * @return True if sensor enabled correctly. False on error.
 */
bool APDS9900::enableLightSensor(bool interrupts)
{

    /* Set default gain, interrupts, enable power, and enable sensor */
    if( !setAmbientLightGain(DEFAULT_AGAIN) ) {
        return false;
    }
    if( interrupts ) {
        if( !setAmbientLightIntEnable(1) ) {
            return false;
        }
    } else {
        if( !setAmbientLightIntEnable(0) ) {
            return false;
        }
    }
    if( !enablePower() ){
        return false;
    }
    if( !setMode(AMBIENT_LIGHT, 1) ) {
        return false;
    }

    return true;

}

/**
 * @brief Ends the light sensor on the APDS-9900
 *
 * @return True if sensor disabled correctly. False on error.
 */
bool APDS9900::disableLightSensor()
{
    if( !setAmbientLightIntEnable(0) ) {
        return false;
    }
    if( !setMode(AMBIENT_LIGHT, 0) ) {
        return false;
    }

    return true;
}

/**
 * @brief Starts the proximity sensor on the APDS-9900
 *
 * @param[in] interrupts true to enable hardware external interrupt on proximity
 * @return True if sensor enabled correctly. False on error.
 */
bool APDS9900::enableProximitySensor(bool interrupts)
{
    /* Set default gain, LED, interrupts, enable power, and enable sensor */
    if( !setProximityGain(DEFAULT_PGAIN) ) {
        return false;
    }
    if( !setLEDDrive(DEFAULT_LDRIVE) ) {
        return false;
    }
    if( interrupts ) {
        if( !setProximityIntEnable(1) ) {
            return false;
        }
    } else {
        if( !setProximityIntEnable(0) ) {
            return false;
        }
    }
    if( !enablePower() ){
        return false;
    }
    if( !setMode(PROXIMITY, 1) ) {
        return false;
    }

    return true;
}

/**
 * @brief Ends the proximity sensor on the APDS-9900
 *
 * @return True if sensor disabled correctly. False on error.
 */
bool APDS9900::disableProximitySensor()
{
	if( !setProximityIntEnable(0) ) {
		return false;
	}
	if( !setMode(PROXIMITY, 0) ) {
		return false;
	}

	return true;
}

/**
 * Turn the APDS-9900 on
 *
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::enablePower()
{
    if( !setMode(POWER, 1) ) {
        return false;
    }

    return true;
}

/**
 * Turn the APDS-9900 off
 *
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::disablePower()
{
    if( !setMode(POWER, 0) ) {
        return false;
    }

    return true;
}

/*******************************************************************************
 * Ambient light and color sensor controls
 ******************************************************************************/

/**
 * @brief Reads the ambient (clear) light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::readAmbientLight(uint16_t &val)
{
    uint8_t val_byte;
    val = 0;

    /* Read value from clear channel, low byte register */
    if( !wireReadDataByte(APDS9900_CDATAL, val_byte) ) {
      Serial.print("CH0 = "); Serial.println(val_byte);
        return false;
    }
    val = val_byte;

    /* Read value from clear channel, high byte register */
    if( !wireReadDataByte(APDS9900_CDATAH, val_byte) ) {
      Serial.print("CH1 = "); Serial.println(val_byte);
        return false;
    }
    val = val + ((uint16_t)val_byte << 8);

    return true;
}

/*******************************************************************************
 * Proximity sensor controls
 ******************************************************************************/

/**
 * @brief Reads the proximity level as an 8-bit value
 *
 * @param[out] val value of the proximity sensor.
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::readProximity(uint8_t &val)
{
  uint8_t val_byte;
  val = 0;

  /* Read value from clear channel, low byte register */
  if( !wireReadDataByte(APDS9900_PDATAL, val_byte) ) {
      Serial.print("APDS9900_PDATAL = "); Serial.println(val_byte);
      return false;
  }
  val = val_byte;

  /* Read value from clear channel, high byte register */
  if( !wireReadDataByte(APDS9900_PDATAH, val_byte) ) {
    Serial.print("APDS9900_PDATAH = "); Serial.println(val_byte);
      return false;
  }
  val = val + ((uint16_t)val_byte << 8);

  return val;
}


/*******************************************************************************
 * Getters and setters for register values
 ******************************************************************************/

/**
 * @brief Returns the lower threshold for proximity detection
 *
 * @return lower threshold
 */
uint8_t APDS9900::getProxIntLowThresh()
{
  uint8_t val_byte;
  int val = 0;

  /* Read value from clear channel, low byte register */
  if( !wireReadDataByte(APDS9900_PILTL, val_byte) ) {
      Serial.print("APDS9900_PILTL = "); Serial.println(val_byte);
      return false;
  }
  val = val_byte;

  /* Read value from clear channel, high byte register */
  if( !wireReadDataByte(APDS9900_PILTH, val_byte) ) {
    Serial.print("APDS9900_PILTH = "); Serial.println(val_byte);
      return false;
  }
  val = val + ((uint16_t)val_byte << 8);

  return true;
}

/**
 * @brief Sets the lower threshold for proximity detection
 *
 * @param[in] threshold the lower proximity threshold
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::setProxIntLowThresh(uint8_t threshold)
{
    if( !wireWriteDataByte(APDS9900_PILTL, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Returns the high threshold for proximity detection
 *
 * @return high threshold
 */
uint8_t APDS9900::getProxIntHighThresh()
{
  uint8_t val_byte;
  int val = 0;

  /* Read value from clear channel, low byte register */
  if( !wireReadDataByte(APDS9900_PIHTL, val_byte) ) {
      Serial.print("APDS9900_PILTL = "); Serial.println(val_byte);
      return false;
  }
  val = val_byte;

  /* Read value from clear channel, high byte register */
  if( !wireReadDataByte(APDS9900_PIHTH, val_byte) ) {
    Serial.print("APDS9900_PILTH = "); Serial.println(val_byte);
      return false;
  }
  val = val + ((uint16_t)val_byte << 8);

  return val;
}

/**
 * @brief Sets the high threshold for proximity detection
 *
 * @param[in] threshold the high proximity threshold
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::setProxIntHighThresh(uint8_t threshold)
{
    if( !wireWriteDataByte(APDS9900_PIHTL, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Returns LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the value of the LED drive strength. 0xFF on failure.
 */
uint8_t APDS9900::getLEDDrive()
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9900_CONTROL, val) ) {
        return ERROR;
    }

    /* Shift and mask out LED drive bits */
    val = (val >> 6) & 0b00000011;

    return val;
}

/**
 * @brief Sets the LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value (0-3) for the LED drive strength
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::setLEDDrive(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9900_CONTROL, val) ) {
        return false;
    }

    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 6;
    val &= 0b00111111;
    val |= drive;

    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9900_CONTROL, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Returns receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the value of the proximity gain. 0xFF on failure.
 */
uint8_t APDS9900::getProximityGain()
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9900_CONTROL, val) ) {
        return ERROR;
    }

    /* Shift and mask out PDRIVE bits */
    val = (val >> 2) & 0b00000011;

    return val;
}

/**
 * @brief Sets the receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::setProximityGain(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9900_CONTROL, val) ) {
        return false;
    }

    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 2;
    val &= 0b11110011;
    val |= drive;

    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9900_CONTROL, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Returns receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @return the value of the ALS gain. 0xFF on failure.
 */
uint8_t APDS9900::getAmbientLightGain()
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9900_CONTROL, val) ) {
        return ERROR;
    }

    /* Shift and mask out ADRIVE bits */
    val &= 0b00000011;

    return val;
}

/**
 * @brief Sets the receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::setAmbientLightGain(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadDataByte(APDS9900_CONTROL, val) ) {
        return false;
    }

    /* Set bits in register to given value */
    drive &= 0b00000011;
    val &= 0b11111100;
    val |= drive;

    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9900_CONTROL, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the low threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9900
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::getLightIntLowThreshold(uint16_t &threshold)
{
    uint8_t val_byte;
    threshold = 0;

    /* Read value from ambient light low threshold, low byte register */
    if( !wireReadDataByte(APDS9900_AILTL, val_byte) ) {
        return false;
    }
    threshold = val_byte;

    /* Read value from ambient light low threshold, high byte register */
    if( !wireReadDataByte(APDS9900_AILTH, val_byte) ) {
        return false;
    }
    threshold = threshold + ((uint16_t)val_byte << 8);

    return true;
}

/**
 * @brief Sets the low threshold for ambient light interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::setLightIntLowThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;

    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;

    /* Write low byte */
    if( !wireWriteDataByte(APDS9900_AILTL, val_low) ) {
        return false;
    }

    /* Write high byte */
    if( !wireWriteDataByte(APDS9900_AILTH, val_high) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the high threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9900
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::getLightIntHighThreshold(uint16_t &threshold)
{
    uint8_t val_byte;
    threshold = 0;

    /* Read value from ambient light high threshold, low byte register */
    if( !wireReadDataByte(APDS9900_AIHTL, val_byte) ) {
        return false;
    }
    threshold = val_byte;

    /* Read value from ambient light high threshold, high byte register */
    if( !wireReadDataByte(APDS9900_AIHTH, val_byte) ) {
        return false;
    }
    threshold = threshold + ((uint16_t)val_byte << 8);

    return true;
}

/**
 * @brief Sets the high threshold for ambient light interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::setLightIntHighThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;

    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;

    /* Write low byte */
    if( !wireWriteDataByte(APDS9900_AIHTL, val_low) ) {
        return false;
    }

    /* Write high byte */
    if( !wireWriteDataByte(APDS9900_AIHTH, val_high) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the low threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9900
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::getProximityIntLowThreshold(uint8_t &threshold)
{
  uint8_t val_byte;
  threshold = 0;

  /* Read value from ambient light high threshold, low byte register */
  if( !wireReadDataByte(APDS9900_PILTL, val_byte) ) {
      return false;
  }
  threshold = val_byte;

  /* Read value from ambient light high threshold, high byte register */
  if( !wireReadDataByte(APDS9900_PILTH, val_byte) ) {
      return false;
  }
  threshold = threshold + ((uint16_t)val_byte << 8);

  return true;
}

/**
 * @brief Sets the low threshold for proximity interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::setProximityIntLowThreshold(uint8_t threshold)
{

    /* Write threshold value to register */
    if( !wireWriteDataByte(APDS9900_PILTL, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the high threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9900
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::getProximityIntHighThreshold(uint8_t &threshold)
{
  uint8_t val_byte;
  threshold = 0;

  /* Read value from ambient light high threshold, low byte register */
  if( !wireReadDataByte(APDS9900_PIHTL, val_byte) ) {
      return false;
  }
  threshold = val_byte;

  /* Read value from ambient light high threshold, high byte register */
  if( !wireReadDataByte(APDS9900_PIHTH, val_byte) ) {
      return false;
  }
  threshold = threshold + ((uint16_t)val_byte << 8);

  return true;
}

/**
 * @brief Sets the high threshold for proximity interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::setProximityIntHighThreshold(uint8_t threshold)
{

    /* Write threshold value to register */
    if( !wireWriteDataByte(APDS9900_PIHTL, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets if ambient light interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t APDS9900::getAmbientLightIntEnable()
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( !wireReadDataByte(APDS9900_ENABLE, val) ) {
        return ERROR;
    }

    /* Shift and mask out AIEN bit */
    val = (val >> 4) & 0b00000001;

    return val;
}

/**
 * @brief Turns ambient light interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::setAmbientLightIntEnable(uint8_t enable)
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( !wireReadDataByte(APDS9900_ENABLE, val) ) {
        return false;
    }

    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 4;
    val &= 0b11101111;
    val |= enable;

    /* Write register value back into ENABLE register */
    if( !wireWriteDataByte(APDS9900_ENABLE, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets if proximity interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t APDS9900::getProximityIntEnable()
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( !wireReadDataByte(APDS9900_ENABLE, val) ) {
        return ERROR;
    }

    /* Shift and mask out PIEN bit */
    val = (val >> 5) & 0b00000001;

    return val;
}

/**
 * @brief Turns proximity interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool APDS9900::setProximityIntEnable(uint8_t enable)
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( !wireReadDataByte(APDS9900_ENABLE, val) ) {
        return false;
    }

    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 5;
    val &= 0b11011111;
    val |= enable;

    /* Write register value back into ENABLE register */
    if( !wireWriteDataByte(APDS9900_ENABLE, val) ) {
        return false;
    }

    return true;
}


/*******************************************************************************
 * Raw I2C Reads and Writes
 ******************************************************************************/

/**
 * @brief Writes a single byte to the I2C device (no register)
 *
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool APDS9900::wireWriteByte(uint8_t val)
{
    Wire.beginTransmission(APDS9900_I2C_ADDR);
    Wire.write(val);
    if( Wire.endTransmission() != 0 ) {
        return false;
    }

    return true;
}

/**
 * @brief Writes a single byte to the I2C device and specified register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool APDS9900::wireWriteDataByte(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(APDS9900_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    if( Wire.endTransmission() != 0 ) {
        return false;
    }

    return true;
}

/**
 * @brief Writes a block (array) of bytes to the I2C device and register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val pointer to the beginning of the data byte array
 * @param[in] len the length (in bytes) of the data to write
 * @return True if successful write operation. False otherwise.
 */
bool APDS9900::wireWriteDataBlock(  uint8_t reg,
                                        uint8_t *val,
                                        unsigned int len)
{
    unsigned int i;

    Wire.beginTransmission(APDS9900_I2C_ADDR);
    Wire.write(reg);
    for(i = 0; i < len; i++) {
        Wire.beginTransmission(val[i]);
    }
    if( Wire.endTransmission() != 0 ) {
        return false;
    }

    return true;
}

/**
 * @brief Reads a single byte from the I2C device and specified register
 *
 * @param[in] reg the register to read from
 * @param[out] the value returned from the register
 * @return True if successful read operation. False otherwise.
 */
bool APDS9900::wireReadDataByte(uint8_t reg, uint8_t &val)
{

    /* Indicate which register we want to read from */
    if (!wireWriteByte(reg)) { // 0x80 |
        return false;
    }

    /* Read from register */
    Wire.requestFrom(APDS9900_I2C_ADDR, 1);
    while (Wire.available()) {
        val = Wire.read();
    }

    return true;
}

/**
 * @brief Reads a single word from the I2C device and specified register(s)
 *
 * @param[in] reg the starting register to read from
 * @param[out] the value returned from the register
 * @return True if successful read operation. False otherwise.
 */
bool APDS9900::wireReadDataWord(uint8_t reg, uint16_t &val)
{

  uint8_t input[2] = {0};

  /* Indicate which register we want to read from */
  if (!wireWriteByte(reg)) {
      return -1;
  }

  //uint8_t val_byte = 0;
  val = 0;

  /* Read block data */
  Wire.requestFrom(APDS9900_I2C_ADDR, 2);
  if (Wire.available()) {
      input[0] = Wire.read();
      input[1] = Wire.read();
  }

  val = input[0];
  val = input[0] + ((uint16_t)input[1] << 8);

  return true;
}

/**
 * @brief Reads a block (array) of bytes from the I2C device and register
 *
 * @param[in] reg the register to read from
 * @param[out] val pointer to the beginning of the data
 * @param[in] len number of bytes to read
 * @return Number of bytes read. -1 on read error.
 */
int APDS9900::wireReadDataBlock(   uint8_t reg,
                                        uint8_t *val,
                                        unsigned int len)
{
    unsigned char i = 0;

    /* Indicate which register we want to read from */
    if (!wireWriteByte(reg)) {
        return -1;
    }

    /* Read block data */
    Wire.requestFrom(APDS9900_I2C_ADDR, len);
    while (Wire.available()) {
        if (i >= len) {
            return -1;
        }
        val[i] = Wire.read();
        i++;
    }

    return i;
}
