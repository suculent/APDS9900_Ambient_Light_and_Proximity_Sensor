/**
 * @file    APDS-9900.h
 * @brief   Library for the SparkFun APDS-9900 breakout board
 * @author  Shawn Hymel (SparkFun Electronics)
 * @author  Matej Sychra (github.com/suculent)
 *
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 *
 * This library interfaces the APDS-9900 to Arduino over I2C. The library
 * relies on the Arduino Wire (I2C) library. to use the library, instantiate an
 * APDS9900 object, call init(), and call the appropriate functions.
 */

#ifndef APDS9900_H
#define APDS9900_H

#include <Arduino.h>

/* Debug */
#define DEBUG                   0

/* APDS-9900 I2C address */
#define APDS9900_I2C_ADDR       0x39

/* Error code for returned values */
#define ERROR                   0xFF

/* Acceptable device IDs */
#define APDS9900_ID_1           0x29
#define APDS9900_ID_2           0xAB    // reserved, this is actually 9960

/* Misc parameters */
#define FIFO_PAUSE_TIME         30      // Wait period (ms) between FIFO reads

/* APDS-9900 register addresses */
#define APDS9900_ENABLE         0x80 // not present yet

#define APDS9900_ATIME          0x01  // ALS integration
#define APDS9900_PTIME          0x02  // Prox integration time
#define APDS9900_WTIME          0x03  // Wait time
#define APDS9900_PPCOUNT        0xE  // pulse count
#define APDS9900_PCONFIG        0xF  // pulse count
#define APDS9900_EN             0x0

#define APDS9900_PDRIVE         0x00  // 100 mA
#define APDS9900_PDIODE         0x20  // CH1 diode
#define APDS9900_PGAIN          0x00  // Proximity Gain x 1
#define APDS9900_AGAIN          0x00  // ALS Gain x 1

#define APDS9900_CDATA0         0xA0 | 0x14
#define APDS9900_CDATA1         0xA0 | 0x16
#define APDS9900_PDATA          0xA0 | 0x18

/* Rest of those defines needs to be aligned with docs. This is just a placeholder copy from APDS-9960. */
#define APDS9900_AILTL          0x84
#define APDS9900_AILTH          0x85
#define APDS9900_AIHTL          0x86
#define APDS9900_AIHTH          0x87
#define APDS9900_PILT           0x89
#define APDS9900_PIHT           0x8B
#define APDS9900_PERS           0x8C
#define APDS9900_CONFIG1        0x8D
#define APDS9900_PPULSE         0x8E
#define APDS9900_CONTROL        0x8F
#define APDS9900_CONFIG2        0x90
#define APDS9900_ID             0x92
#define APDS9900_STATUS         0x93
#define APDS9900_POFFSET_UR     0x9D
#define APDS9900_POFFSET_DL     0x9E
#define APDS9900_CONFIG3        0x9F
#define APDS9900_GPENTH         0xA0
#define APDS9900_GEXTH          0xA1
#define APDS9900_GCONF1         0xA2
#define APDS9900_GCONF2         0xA3
#define APDS9900_GOFFSET_U      0xA4
#define APDS9900_GOFFSET_D      0xA5
#define APDS9900_GOFFSET_L      0xA7
#define APDS9900_GOFFSET_R      0xA9
#define APDS9900_GPULSE         0xA6
#define APDS9900_GCONF3         0xAA
#define APDS9900_GCONF4         0xAB
#define APDS9900_GFLVL          0xAE
#define APDS9900_GSTATUS        0xAF
#define APDS9900_IFORCE         0xE4
#define APDS9900_PICLEAR        0xE5
#define APDS9900_CICLEAR        0xE6
#define APDS9900_AICLEAR        0xE7
#define APDS9900_GFIFO_U        0xFC
#define APDS9900_GFIFO_D        0xFD
#define APDS9900_GFIFO_L        0xFE
#define APDS9900_GFIFO_R        0xFF

/* Bit fields */
#define APDS9900_PON            0b00000001
#define APDS9900_AEN            0b00000010
#define APDS9900_PEN            0b00000100
#define APDS9900_WEN            0b00001000
#define APSD9900_AIEN           0b00010000
#define APDS9900_PIEN           0b00100000
#define APDS9900_GEN            0b01000000
#define APDS9900_GVALID         0b00000001

/* On/Off definitions */
#define OFF                     0
#define ON                      1

/* Acceptable parameters for setMode */
#define POWER                   0
#define AMBIENT_LIGHT           1
#define PROXIMITY               2
#define WAIT                    3
#define AMBIENT_LIGHT_INT       4
#define PROXIMITY_INT           5
#define ALL                     6

/* LED Drive values */
#define LED_DRIVE_100MA         0
#define LED_DRIVE_50MA          1
#define LED_DRIVE_25MA          2
#define LED_DRIVE_12_5MA        3

/* Proximity Gain (PGAIN) values */
#define PGAIN_1X                0
#define PGAIN_2X                1
#define PGAIN_4X                2
#define PGAIN_8X                3

/* ALS Gain (AGAIN) values */
#define AGAIN_1X                0
#define AGAIN_4X                1
#define AGAIN_16X               2
#define AGAIN_64X               3

/* LED Boost values */
#define LED_BOOST_100           0
#define LED_BOOST_150           1
#define LED_BOOST_200           2
#define LED_BOOST_300           3

/* Default values */
#define DEFAULT_ATIME           0xFF   // or 219     // 103ms
#define DEFAULT_WTIME           0xFF    // or 246     // 27ms
#define DEFAULT_PTIME           0xFF    // 2.7 ms – minimum Prox integration time
#define DEFAULT_PPCOUNT         0x01    // Minimum prox pulse count
#define DEFAULT_PROX_PPULSE     0x87    // 16us, 8 pulses
#define DEFAULT_POFFSET_UR      0       // 0 offset
#define DEFAULT_POFFSET_DL      0       // 0 offset
#define DEFAULT_CONFIG1         0x60    // No 12x wait (WTIME) factor
#define DEFAULT_LDRIVE          LED_DRIVE_100MA
#define DEFAULT_PGAIN           PGAIN_4X
#define DEFAULT_AGAIN           AGAIN_4X
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_AILT            0xFFFF  // Force interrupt for calibration
#define DEFAULT_AIHT            0
#define DEFAULT_PERS            0x11    // 2 consecutive prox or ALS for int.
#define DEFAULT_CONFIG2         0x01    // No saturation interrupts or LED boost
#define DEFAULT_CONFIG3         0       // Enable all photodiodes, no SAI
#define DEFAULT_GLDRIVE         LED_DRIVE_100MA

/* State definitions */
enum {
  NAK_STATE,
  NEAR_STATE,
  FAR_STATE,
  ALL_STATE
};

/* APDS9900 Class */
class APDS9900 {
public:

    /* Initialization methods */
    APDS9900();
    ~APDS9900();
    bool init();
    uint8_t getMode();
    bool setMode(uint8_t mode, uint8_t enable);

    /* Turn the APDS-9900 on and off */
    bool enablePower();
    bool disablePower();

    /* Enable or disable specific sensors */
    bool enableLightSensor(bool interrupts = false);
    bool disableLightSensor();
    bool enableProximitySensor(bool interrupts = false);
    bool disableProximitySensor();

    /* LED drive strength control */
    uint8_t getLEDDrive();
    bool setLEDDrive(uint8_t drive);

    /* Gain control */
    uint8_t getAmbientLightGain();
    bool setAmbientLightGain(uint8_t gain);
    uint8_t getProximityGain();
    bool setProximityGain(uint8_t gain);

    /* Get and set light interrupt thresholds */
    bool getLightIntLowThreshold(uint16_t &threshold);
    bool setLightIntLowThreshold(uint16_t threshold);
    bool getLightIntHighThreshold(uint16_t &threshold);
    bool setLightIntHighThreshold(uint16_t threshold);

    /* Get and set proximity interrupt thresholds */
    bool getProximityIntLowThreshold(uint8_t &threshold);
    bool setProximityIntLowThreshold(uint8_t threshold);
    bool getProximityIntHighThreshold(uint8_t &threshold);
    bool setProximityIntHighThreshold(uint8_t threshold);

    /* Get and set interrupt enables */
    uint8_t getAmbientLightIntEnable();
    bool setAmbientLightIntEnable(uint8_t enable);
    uint8_t getProximityIntEnable();
    bool setProximityIntEnable(uint8_t enable);

    /* Clear interrupts */
    bool clearAmbientLightInt();
    bool clearProximityInt();

    /* Ambient light methods */
    bool readAmbientLight(uint16_t &val);

    /* Proximity methods */
    bool readProximity(uint8_t &val);

private:

    /* Proximity Interrupt Threshold */
    uint8_t getProxIntLowThresh();
    bool setProxIntLowThresh(uint8_t threshold);
    uint8_t getProxIntHighThresh();
    bool setProxIntHighThresh(uint8_t threshold);

    /* LED Boost Control */
    uint8_t getLEDBoost();
    bool setLEDBoost(uint8_t boost);

    /* Proximity photodiode select */
    uint8_t getProxGainCompEnable();
    bool setProxGainCompEnable(uint8_t enable);
    uint8_t getProxPhotoMask();
    bool setProxPhotoMask(uint8_t mask);

    /* Raw I2C Commands */
    bool wireWriteByte(uint8_t val);
    bool wireWriteDataByte(uint8_t reg, uint8_t val);
    bool wireWriteDataBlock(uint8_t reg, uint8_t *val, unsigned int len);
    bool wireReadDataByte(uint8_t reg, uint8_t &val);
    int wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len);

};

#endif
