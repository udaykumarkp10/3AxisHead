/*
 * adxl_345.h
 *
 *  Created on: May 30, 2024
 *      Author: udaykumar
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include <project.h>

// ADXL345 Register Addresses
#define ADXL_REG_DEVID          0x00
#define ADXL_REG_THRESH_TAP     0x1D
#define ADXL_REG_OFSX           0x1E
#define ADXL_REG_OFSY           0x1F
#define ADXL_REG_OFSZ           0x20
#define ADXL_REG_DUR            0x21
#define ADXL_REG_LATENT         0x22
#define ADXL_REG_WINDOW         0x23
#define ADXL_REG_THRESH_ACT     0x24
#define ADXL_REG_THRESH_INACT   0x25
#define ADXL_REG_TIME_INACT     0x26
#define ADXL_REG_ACT_INACT_CTL  0x27
#define ADXL_REG_THRESH_FF      0x28
#define ADXL_REG_TIME_FF        0x29
#define ADXL_REG_TAP_AXES       0x2A
#define ADXL_REG_ACT_TAP_STATUS 0x2B
#define ADXL_REG_BW_RATE        0x2C
#define ADXL_REG_POWER_CTL      0x2D
#define ADXL_REG_INT_ENABLE     0x2E
#define ADXL_REG_INT_MAP        0x2F
#define ADXL_REG_INT_SOURCE     0x30
#define ADXL_REG_DATA_FORMAT    0x31
#define ADXL_REG_DATAX0         0x32
#define ADXL_REG_DATAX1         0x33
#define ADXL_REG_DATAY0         0x34
#define ADXL_REG_DATAY1         0x35
#define ADXL_REG_DATAZ0         0x36
#define ADXL_REG_DATAZ1         0x37
#define ADXL_REG_FIFO_CTL       0x38
#define ADXL_REG_FIFO_STATUS    0x39

// ADXL345 SPI Commands
#define ADXL_SPI_READ           0x80
#define ADXL_SPI_MULTI_BYTE     0x40
#define ADXL_SPI_INCREMENT      0x20

// ADXL345 Full Resolution Mode
#define ADXL_FULL_RES           0x08
#define ADXL345_SCALE_FACTOR_4G 0.0078

// ADXL345 SPI Rate
#define ADXL_SPI_RATE_3_125HZ   0x00
#define ADXL_SPI_RATE_6_25HZ    0x01
#define ADXL_SPI_RATE_12_5HZ    0x02
#define ADXL_SPI_RATE_25HZ      0x03
#define ADXL_SPI_RATE_50HZ      0x04
#define ADXL_SPI_RATE_100HZ     0x05
#define ADXL_SPI_RATE_200HZ     0x06
#define ADXL_SPI_RATE_400HZ     0x07

// ADXL345 SPI Range
#define ADXL_RANGE_2G           0x00
#define ADXL_RANGE_4G           0x01
#define ADXL_RANGE_8G           0x02
#define ADXL_RANGE_16G          0x03

// ADXL345 Resolution
#define ADXL_RES_10BIT          0x00
#define ADXL_RES_8BIT           0x01

// ADXL345 Justify
#define ADXL_JUSTIFY_RIGHT      0x00
#define ADXL_JUSTIFY_LEFT       0x01

// ADXL345 Auto Sleep Mode
#define ADXL_AUTO_SLEEP_OFF     0x00
#define ADXL_AUTO_SLEEP_ON      0x01

// ADXL345 Link Mode
#define ADXL_LINK_MODE_OFF      0x00
#define ADXL_LINK_MODE_ON       0x01

// ADXL345 Data Rate in Hz
#define ADXL_DATA_RATE_3200HZ   3200
#define ADXL_DATA_RATE_1600HZ   1600
#define ADXL_DATA_RATE_800HZ    800
#define ADXL_DATA_RATE_400HZ    400
#define ADXL_DATA_RATE_200HZ    200
#define ADXL_DATA_RATE_100HZ    100
#define ADXL_DATA_RATE_50HZ     50
#define ADXL_DATA_RATE_25HZ     25
#define ADXL_DATA_RATE_12_5HZ   12.5
#define ADXL_DATA_RATE_6_25HZ   6.25
#define ADXL_DATA_RATE_3_13HZ   3.13

// ADXL345 Output Resolution in mg/LSB
#define ADXL_RESOLUTION_2G      4
#define ADXL_RESOLUTION_4G      8
#define ADXL_RESOLUTION_8G      16
#define ADXL_RESOLUTION_16G     32

// ADXL345 Output Data Range in g
#define ADXL_RANGE_2G_VAL       2
#define ADXL_RANGE_4G_VAL       4
#define ADXL_RANGE_8G_VAL       8
#define ADXL_RANGE_16G_VAL      16

// ADXL345 SPI Mode
#define ADXL_SPI_MODE_4WIRE     0
#define ADXL_SPI_MODE_3WIRE     1

#define ACCEL_SCALING_FACTOR 10000

bool Adxl345_init(void);
void adxl_write (uint8_t address, uint8_t value);
void adxl_read(uint8_t address, uint8_t* data_rec);


int32_t getAcceleration(char axis);


#endif /* INC_ADXL345_H_ */
