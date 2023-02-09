
#ifndef __QMI8653_H__
#define __QMI8653_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#if !(defined(ESP8266)             || defined(ESP32) || \
      defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2) || \
      defined(ARDUINO_ARCH_STM32)  || defined(__ASR6501__) || \
      defined(ARDUINO_ARCH_NRF52))
#include <Adafruit_Sensor.h>
#endif

#ifdef __AVR_ATtiny85__
#include "TinyWireM.h"
#define Wire TinyWireM
#else
#include <Wire.h>
#endif

/*=========================================================================
    I2C ADDRESS/BITS/SETTINGS
    -----------------------------------------------------------------------*/
#define QMI8653_ADDRESS                (0x6B)
#define QMI8653_CHIPID                 (0x05)
// #define QMI8653_CHIPID                 (0x60)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum {

    QMI8653_ADD = 0x6B,
    QMI8653_WHO_AM_I = 0X00,  //Device identifier
    QMI8653_REVISION_ID = 0x01,
    QMI8653_CTRL1 = 0x02,   //Serial Interface and Sensor Enable
    QMI8653_CTRL2 = 0x03,   //Accelerometer Settings
    QMI8653_CTRL3 = 0x04,   //Gyroscope Settings
    QMI8653_CTRL4 = 0X05,   //Magnetometer Settings
    QMI8653_CTRL5 = 0X06,   //Sensor Data Processing Settings
    QMI8653_CTRL7 = 0x08,   //Enable Sensors and Configure Data Reads
    QMI8653_CTRL8 = 0X09,   //Reserved – Special Settings

///<Sensor Data Output Registers>
    QMI8653_AccX_L = 0x35,
    QMI8653_AccX_H = 0x36,
    QMI8653_AccY_L = 0x37,
    QMI8653_AccY_H = 0x38,
    QMI8653_AccZ_L = 0x39,
    QMI8653_AccZ_H = 0x3A,
    QMI8653_TEMP_L = 0x33,

    QMI8653_GyrX_L = 0x3B,
    QMI8653_GyrX_H = 0x3C,
    QMI8653_GyrY_L = 0x3D,
    QMI8653_GyrY_H = 0x3E,
    QMI8653_GyrZ_L = 0x3F,
    QMI8653_GyrZ_H = 0x40
};


class QMI8653
{
public:
    QMI8653();
    QMI8653(int8_t cspin);
    QMI8653(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

    bool  begin(uint8_t addr = QMI8653_ADDRESS, uint8_t chipid = QMI8653_CHIPID);

    void readCoefficients(void);
    uint8_t spixfer(uint8_t x);

    uint8_t readAcceleration(uint16_t *x_value, uint16_t  *y_value, uint16_t *z_value);
    uint8_t readAngular(uint16_t *x_value, uint16_t  *y_value, uint16_t *z_value);

    void      write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian

    uint8_t   _i2caddr;
    int32_t   _sensorID;
    int32_t t_fine;

    int8_t _cs, _mosi, _miso, _sck;
private:


};

#endif
