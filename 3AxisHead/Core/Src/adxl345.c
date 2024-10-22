/*
 * adxl_345.c
 *
 *  Created on: May 30, 2024
 *      Author: udaykumar
 */


#include <adxl345.h>

void adxl_write (uint8_t address, uint8_t value)
{
	uint8_t data[2];
	data[0] = address | ADXL_SPI_MULTI_BYTE;  // multibyte write enabled
	data[1] = value;
	ADXL345_CS_RESET();     // pull the cs pin low to enable the slave
	HAL_SPI_Transmit(&ADXL345_PORT, data, 2, 100);  // transmit the address and data
	ADXL345_CS_SET(); // pull the cs pin high to disable the slave
}


void adxl_read(uint8_t address, uint8_t* data_rec) {
    address |= ADXL_SPI_READ;  // read operation
    address |= ADXL_SPI_MULTI_BYTE;  // multibyte read
    ADXL345_CS_RESET();  // pull the cs pin low to enable the slave
    HAL_SPI_Transmit(&ADXL345_PORT, &address, 1, 100);  // send the address from where you want to read data
    HAL_SPI_Receive(&ADXL345_PORT, data_rec, 6, 100);  // read 6 bytes of data
    ADXL345_CS_SET();  // pull the cs pin high to disable the slave
}

bool Adxl345_init(void)
{
	adxl_write (ADXL_REG_DATA_FORMAT, ADXL_RANGE_4G);  // data_format range= +- 4g
	adxl_write (ADXL_REG_POWER_CTL, ADXL_RES_10BIT);  // reset all bits
	adxl_write (ADXL_REG_POWER_CTL, ADXL_FULL_RES);  // power_cntl measure and wake up 8hz
	return true;
}

int32_t getAcceleration(char axis) {
    uint8_t accel_data[6];  // Buffer to store raw acceleration data from the ADXL345
    int16_t raw_accel_x, raw_accel_y, raw_accel_z;  // Raw acceleration values for the X, Y, and Z axes
    int16_t raw_accel;  // Declare raw_accel to store the selected axis value

    // Read acceleration data from ADXL345
    adxl_read(ADXL_REG_DATAX0, accel_data);

    raw_accel_x = ((accel_data[1] << 8) | accel_data[0]);
    raw_accel_y = ((accel_data[3] << 8) | accel_data[2]);
    raw_accel_z = ((accel_data[5] << 8) | accel_data[4]);

    // Select the correct axis based on the input character 'X', 'Y', or 'Z'
    switch (axis) {
        case 'X':
            raw_accel = raw_accel_x;
            break;
        case 'Y':
            raw_accel = raw_accel_y;
            break;
        case 'Z':
            raw_accel = raw_accel_z;
            break;
        default:
            break;
    }

    // Convert the raw value to g and apply offset encoding to store as uint32_t
    int32_t accel_g = ((raw_accel * ADXL345_SCALE_FACTOR_4G) * ACCEL_SCALING_FACTOR);

    return accel_g;
}
