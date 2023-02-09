
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "QMI8658.h"


QMI8653::QMI8653()
    : _cs(-1), _mosi(-1), _miso(-1), _sck(-1)
{ }

QMI8653::QMI8653(int8_t cspin)
    : _cs(cspin), _mosi(-1), _miso(-1), _sck(-1)
{ }

QMI8653::QMI8653(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
    : _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin)
{ }


bool QMI8653::begin(uint8_t a, uint8_t chipid)
{
    _i2caddr = a;

    if (_cs == -1) {
        // i2c
        Wire.begin();
    } else {
        digitalWrite(_cs, HIGH);
        pinMode(_cs, OUTPUT);

        if (_sck == -1) {
            // hardware SPI
            SPI.begin();
        } else {
            // software SPI
            pinMode(_sck, OUTPUT);
            pinMode(_mosi, OUTPUT);
            pinMode(_miso, INPUT);
        }
    }
    Serial.println(read8(QMI8653_WHO_AM_I));
    if (read8(QMI8653_WHO_AM_I) != chipid)
        return false;


    write8(QMI8653_CTRL1, 0x40); //Serial Interface and Sensor Enable<串行接口（SPI或I 2 C）地址自动递增>
    write8(QMI8653_CTRL7, 0x03); //Enable Sensors and Configure Data Reads<Enable Gyroscope Accelerometer>

    write8(QMI8653_CTRL2, 0x04); //Accelerometer Settings<±2g  500Hz>
    write8(QMI8653_CTRL3, 0x64); //Gyroscope Settings< ±2048dps 500Hz>
    write8(QMI8653_CTRL5, 0x11); //Sensor Data Processing Settings<Enable Gyroscope Accelerometer 低通滤波>

    return true;
}


uint8_t QMI8653::spixfer(uint8_t x)
{
    if (_sck == -1)
        return SPI.transfer(x);

    // software spi
    //Serial.println("Software SPI");
    uint8_t reply = 0;
    for (int i = 7; i >= 0; i--) {
        reply <<= 1;
        digitalWrite(_sck, LOW);
        digitalWrite(_mosi, x & (1 << i) ? HIGH : LOW);
        digitalWrite(_sck, HIGH);
        if (digitalRead(_miso))
            reply |= 1;
    }
    return reply;
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C/SPI
*/
/**************************************************************************/
void QMI8653::write8(byte reg, byte value)
{
    if (_cs == -1) {
        Wire.beginTransmission((uint8_t)_i2caddr);
        Wire.write((uint8_t)reg);
        Wire.write((uint8_t)value);
        Wire.endTransmission();
    } else {
#if defined(SPI_HAS_TRANSACTION)
        if (_sck == -1)
            SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
#endif
        digitalWrite(_cs, LOW);
        spixfer(reg & ~0x80); // write, bit 7 low
        spixfer(value);
        digitalWrite(_cs, HIGH);
#if defined(SPI_HAS_TRANSACTION)
        if (_sck == -1)
            SPI.endTransaction();              // release the SPI bus
#endif
    }
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C/SPI
*/
/**************************************************************************/
uint8_t QMI8653::read8(byte reg)
{
    uint8_t value;

    if (_cs == -1) {
        Wire.beginTransmission((uint8_t)_i2caddr);
        Wire.write((uint8_t)reg);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)_i2caddr, (byte)1);
        value = Wire.read();

    } else {
#if defined(SPI_HAS_TRANSACTION)
        if (_sck == -1)
            SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
#endif
        digitalWrite(_cs, LOW);
        spixfer(reg | 0x80); // read, bit 7 high
        value = spixfer(0);
        digitalWrite(_cs, HIGH);
#if defined(SPI_HAS_TRANSACTION)
        if (_sck == -1)
            SPI.endTransaction();              // release the SPI bus
#endif
    }
    return value;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C/SPI
*/
/**************************************************************************/
uint16_t QMI8653::read16(byte reg)
{
    uint16_t value;

    if (_cs == -1) {
        Wire.beginTransmission((uint8_t)_i2caddr);
        Wire.write((uint8_t)reg);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)_i2caddr, (byte)2);
        value = (Wire.read() << 8) | Wire.read();

    } else {
#if defined(SPI_HAS_TRANSACTION)
        if (_sck == -1)
            SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
#endif
        digitalWrite(_cs, LOW);
        spixfer(reg | 0x80); // read, bit 7 high
        value = (spixfer(0) << 8) | spixfer(0);
        digitalWrite(_cs, HIGH);
#if defined(SPI_HAS_TRANSACTION)
        if (_sck == -1)
            SPI.endTransaction();              // release the SPI bus
#endif
    }

    return value;
}

uint16_t QMI8653::read16_LE(byte reg)
{
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);

}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C/SPI
*/
/**************************************************************************/
int16_t QMI8653::readS16(byte reg)
{
    return (int16_t)read16(reg);

}

int16_t QMI8653::readS16_LE(byte reg)
{
    return (int16_t)read16_LE(reg);

}


/**************************************************************************/
/*!
    @brief  Reads a 24 bit value over I2C/SPI
*/
/**************************************************************************/
uint32_t QMI8653::read24(byte reg)
{
    uint32_t value;

    if (_cs == -1) {
        Wire.beginTransmission((uint8_t)_i2caddr);
        Wire.write((uint8_t)reg);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)_i2caddr, (byte)3);

        value = Wire.read();
        value <<= 8;
        value |= Wire.read();
        value <<= 8;
        value |= Wire.read();

    } else {
#if defined(SPI_HAS_TRANSACTION)
        if (_sck == -1)
            SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
#endif
        digitalWrite(_cs, LOW);
        spixfer(reg | 0x80); // read, bit 7 high

        value = spixfer(0);
        value <<= 8;
        value |= spixfer(0);
        value <<= 8;
        value |= spixfer(0);

        digitalWrite(_cs, HIGH);
#if defined(SPI_HAS_TRANSACTION)
        if (_sck == -1)
            SPI.endTransaction();              // release the SPI bus
#endif
    }

    return value;
}


uint8_t QMI8653::readAcceleration(uint16_t *x_value, uint16_t  *y_value, uint16_t *z_value)
{

    *x_value = read16(QMI8653_AccX_L) ;
    *y_value = read16(QMI8653_AccY_L);
    *z_value = read16(QMI8653_AccZ_L);

    return 0;
}
uint8_t QMI8653::readAngular(uint16_t *x_value, uint16_t  *y_value, uint16_t *z_value)
{

    *x_value = read16(QMI8653_AccX_L) ;
    *y_value = read16(QMI8653_AccY_L);
    *z_value = read16(QMI8653_AccZ_L);

    return 0;
}