#include "INA226.hpp"

INA226::INA226(I2C_HandleTypeDef* hi2c, uint8_t address)
{

	this->_i2c.I2cClassInit(hi2c);
	this->_address = address;
	this->updateConfiguration();
}

void INA226::updateConfiguration()
{
	uint16_t reg = 0x00 + (_reset << 15);

	reg |= (_average << 9);
	reg |= (_convTime << 6);
	reg |= (_convTime << 3);
	reg |= _mode;

	this->_i2c.write2Bytes(_address, CONFIG_REG, reg);

	this->calibrateDevice();
}

/***
 * @brief update INA226 address
 * @param new_add
 * @note
 * @note	A1 		A0 	SLAVE_ADDRESS
 * @note    GND 	GND 1000000
 * @note    GND 	VS 	1000001
 * @note    GND 	SDA 1000010
 * @note    GND 	SCL 1000011
 * @note    VS 		GND 1000100
 * @note	VS 		VS 	1000101
 * @note	VS 		SDA 1000110
 * @note	VS 		SCL 1000111
 * @note	SDA 	GND 1001000
 * @note	SDA 	VS 	1001001
 * @note	SDA 	SDA 1001010
 * @note	SDA 	SCL 1001011
 * @note	SCL 	GND 1001100
 * @note	SCL 	VS 	1001101
 * @note	SCL 	SDA 1001110
 * @note	SCL 	SCL 1001111
*/
void INA226::updateSensorAdd(uint8_t new_add)
{
	this->_address = new_add;
}


float INA226::getShuntVol()
{
	uint8_t data[2] = {0x00, 0x00};
	this->_i2c.readMultiBytes(_address, SHUNTVOL_REG, data, 2);

	return (float)((int16_t)((data[0] << 8) + data[1]) * _voltageResolutionShunt);
}

float INA226::getBusVol()
{
	uint8_t data[2] = {0x00, 0x00};
	this->_i2c.readMultiBytes(_address, BUSVOL_REG, data, 2);

	return (float)(((data[0] << 8) + data[1]) * _voltageResolutionBus);
}

float INA226::getPower()
{
	uint8_t data[2] = {0x00, 0x00};
	this->_i2c.readMultiBytes(_address, POWER_REG, data, 2);

	return (float)(((data[0] << 8) + data[1]) * 25 * _currentResolution);
}

float INA226::getCurrent()
{
	uint8_t data[2] = {0x00, 0x00};
	this->_i2c.readMultiBytes(_address, CURRENT_REG, data, 2);

	return (float)(((data[0] << 8) + data[1]) * _currentResolution);
}

void INA226::calibrateDevice()
{
	//Calibration for Current Measurement
	//resolution = maxCurrent / pow(2, 15) =approx 500uA/Bit;

	//_cal = (uint16_t)(0.00512 / (_currentResolution * _shuntResistance));

	this->_i2c.write2Bytes(_address, CALIB_REG, _cal);

}


