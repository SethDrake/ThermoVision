
#include "mlx90614.h"
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"

using namespace std;

MLX90614::MLX90614() {

}

MLX90614::~MLX90614() {

}

void MLX90614::setup(I2C_TypeDef* i2cPort) {
	this->mlxI2C = i2cPort;
}

void MLX90614::i2cWrite(uint8_t i2cAddress, uint8_t* pBuffer, uint8_t NumByteToWrite)
{
	I2C_AcknowledgeConfig(mlxI2C, ENABLE);
	/* Send START condition */
	I2C_GenerateSTART(mlxI2C, ENABLE);
    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(mlxI2C, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send I2C address for write */
    I2C_Send7bitAddress(mlxI2C, i2cAddress << 1, I2C_Direction_Transmitter);
    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(mlxI2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	while (NumByteToWrite)
	{
		I2C_SendData(mlxI2C, *pBuffer);
		/* Test on EV8 and clear it */
		while (!I2C_CheckEvent(mlxI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
		if (NumByteToWrite > 0) {
			/* Point to the next location where the byte write will be saved */
			pBuffer++;
			/* Decrement the read bytes counter */
			NumByteToWrite--;
		}
	}
}

void MLX90614::i2cRead(uint8_t i2cAddress, uint8_t* pBuffer, uint8_t NumByteToRead)
{
	/* Send START condition */
	I2C_GenerateSTART(mlxI2C, ENABLE);
	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(mlxI2C, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send I2C address for read */
    I2C_Send7bitAddress(mlxI2C, i2cAddress << 1, I2C_Direction_Receiver);
    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(mlxI2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    /* While there is data to be read */
    while(NumByteToRead)
    {
        if(NumByteToRead == 1)
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(mlxI2C, DISABLE);
            /* Send STOP Condition */
            I2C_GenerateSTOP(mlxI2C, ENABLE);
        }

        /* Test on EV7 and clear it */
        if(I2C_CheckEvent(mlxI2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the I2C device */
            *pBuffer = I2C_ReceiveData(mlxI2C);
            /* Point to the next location where the byte read will be saved */
            pBuffer++;
            /* Decrement the read bytes counter */
            NumByteToRead--;
        }
    }

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(mlxI2C, ENABLE);
}

uint8_t MLX90614::crc8(uint8_t *p, uint8_t len)
{
	uint16_t i;
	uint16_t crc = 0x0;

	while (len--) {
		i = (crc ^ *p++) & 0xFF;
		crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
	}
	return crc;
}

uint16_t MLX90614::readRam(uint8_t regAddress)
{
	uint16_t result;
	uint8_t buf[3];
	
	i2cWrite(MLX90614_ADDR, &regAddress, 1);
	i2cRead(MLX90614_ADDR, buf, 3);
	result = ((buf[1] & 0x007F) << 8) + buf[0];
	return result;
}

uint16_t MLX90614::readEeprom(uint8_t regAddress)
{
	uint16_t result;
	uint8_t buf[3];

	i2cWrite(MLX90614_ADDR, &regAddress, 1);
	i2cRead(MLX90614_ADDR, buf, 3);
	result = (buf[1] << 8) + buf[0];
	return result;
}

void MLX90614::writeEeprom(uint8_t regAddress, uint16_t val)
{
	uint16_t result;
	uint8_t buf[4];
	uint8_t pec;

	buf[0] = MLX90614_ADDR << 1;
	buf[1] = regAddress;
	buf[2] = val & 0x00FF;
	buf[3] = val >> 8;
	pec = crc8(buf, 4);

	buf[0] = regAddress;
	buf[1] = val & 0x00FF;
	buf[2] = val >> 8;
	buf[3] = pec;
	i2cWrite(MLX90614_ADDR, buf, 4);
}


float MLX90614::readTempKelvin(short sensorNum)
{
	uint8_t addr = (sensorNum == 0) ? 0x07 : 0x08;
	return (float) (readRam(addr) * 0.02) - 0.01;
}

float MLX90614::readTemp(short sensorNum)
{
    return readTempKelvin(sensorNum) - 273.15;  //Convert kelvin to degree Celsius
}


uint16_t MLX90614::rgb2color(uint8_t R, uint8_t G, uint8_t B)
{
	return ((R & 0xF8) << 8) | ((G & 0xFC) << 3) | (B >> 3);
}

uint8_t MLX90614::calculateRGB(uint8_t rgb1, uint8_t rgb2, float t1, float step, float t) {
	return (uint8_t)(rgb1 + (((t - t1) / step) * (rgb2 - rgb1)));
}

uint16_t MLX90614::temperatureToRGB565(float temperature, float minTemp, float maxTemp) {
	uint8_t r, g, b;

	uint16_t val = rgb2color(DEFAULT_COLOR_SCHEME[0][0], DEFAULT_COLOR_SCHEME[0][1], DEFAULT_COLOR_SCHEME[0][2]);
	if (temperature < minTemp) {
		val = rgb2color(DEFAULT_COLOR_SCHEME[0][0], DEFAULT_COLOR_SCHEME[0][1], DEFAULT_COLOR_SCHEME[0][2]);
	}
	else if (temperature >= maxTemp) {
		short colorSchemeSize = sizeof(DEFAULT_COLOR_SCHEME);
		val = rgb2color(DEFAULT_COLOR_SCHEME[colorSchemeSize - 1][0], DEFAULT_COLOR_SCHEME[colorSchemeSize - 1][1], DEFAULT_COLOR_SCHEME[colorSchemeSize - 1][2]);
	}
	else {
		float step = (maxTemp - minTemp) / 10.0;
		uint8_t step1 = (uint8_t)((temperature - minTemp) / step);
		uint8_t step2 = step1 + 1;
		uint8_t red = calculateRGB(DEFAULT_COLOR_SCHEME[step1][0], DEFAULT_COLOR_SCHEME[step2][0], (minTemp + step1 * step), step, temperature);
		uint8_t green = calculateRGB(DEFAULT_COLOR_SCHEME[step1][1], DEFAULT_COLOR_SCHEME[step2][1], (minTemp + step1 * step), step, temperature);
		uint8_t blue = calculateRGB(DEFAULT_COLOR_SCHEME[step1][2], DEFAULT_COLOR_SCHEME[step2][2], (minTemp + step1 * step), step, temperature);
		val = rgb2color(red, green, blue);
	}
	return val;
}

