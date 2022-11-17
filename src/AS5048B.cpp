extern "C"
{
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <i2c/smbus.h>
}

#include "AS5048B.h"


AS5048B::AS5048B(void) {
    _chipAddress = AS5048_ADDRESS;
    _debugFlag = false;
}

AS5048B::AS5048B(uint8_t chipAddress) {
    _chipAddress = chipAddress;
    _debugFlag = false;
}


void AS5048B::setI2cNumber() {
    snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
}

void AS5048B::setAdapter(int num) {
    adapter_nr = num;
}

void AS5048B:: openConnection() {
    file = open(filename, O_RDWR);
    if (file < 0) {
        printf("Error While Opening I2C connection : 1, Error Number: %d", errno);
        exit(1);
    }

    _clockWise = false;
    _lastAngleRaw = 0.0;
    _zeroRegVal = AS5048B::zeroRegR();
    _addressRegVal = AS5048B::addressRegR();

    AS5048B::resetMovingAvgExp();
}                                                      

void AS5048B::setDeviceAddress(int address){
    addr = address;
}

void AS5048B::setRegister(uint8_t regstr){
    _chipAddress = regstr;
}

uint8_t AS5048B::readReg8(uint8_t addr){


    if (ioctl(file, I2C_SLAVE, _chipAddress) < 0) {
        printf("Error While Opening I2C connection : 2, Error Number: %d", errno);
        exit(1);
    }

    /* Using SMBus commands */
    _addressRegVal = i2c_smbus_read_byte_data(file, addr);
    if (_addressRegVal < 0) {
        printf("Error While reading I2C data, Error Number: %d", errno);
        exit(1);
    } else {
        return _addressRegVal;
    }
}


uint16_t AS5048B::readReg16(uint8_t addr){

    if (ioctl(file, I2C_SLAVE, _chipAddress) < 0) {
        printf("Error While Opening I2C connection : 3, Error Number: %d", errno);
        exit(1);
    }

	uint8_t requestResult;
	uint8_t readArray[2];
	uint16_t readValue = 0;
    /* Using SMBus commands */
    readArray[0] = i2c_smbus_read_byte_data(file, addr);
	readArray[1] = i2c_smbus_read_byte_data(file, addr);
	readValue = (((uint16_t) readArray[0]) << 6);
	readValue += (readArray[1] & 0x3F);
    if (readArray[0] < 0) {
        printf("Error While reading I2C data, Error Number: %d", errno);
        exit(1);
    } else {
        return readValue;
    }
}

void AS5048B::writeReg(uint8_t addr, uint8_t value)
{
    if (ioctl(file, I2C_SLAVE, _chipAddress) < 0) {
        printf("Error While Opening I2C connection : 4, Error Number: %d", errno);
        exit(1);
    }

    _addressRegVal = i2c_smbus_write_byte_data(file, addr, value);
    if (_addressRegVal < 0) {
        printf("Error While reading I2C data, Error Number: %d", errno);
        exit(1);
    }
}

double AS5048B::convertAngle(int unit, double angle) {
    // convert raw sensor reading into angle unit
    double angleConv;

    switch (unit)
    {
    case U_RAW:
        // Sensor raw measurement
        angleConv = angle;
        break;
    case U_TRN:
        // full turn ratio
        angleConv = (angle / AS5048B_RESOLUTION);
        break;
    case U_DEG:
        // degree
        angleConv = (angle / AS5048B_RESOLUTION) * 360.0;
        break;
    case U_RAD:
        // Radian
        angleConv = (angle / AS5048B_RESOLUTION) * 2 * 3.14;
        break;
    case U_MOA:
        // minute of arc
        angleConv = (angle / AS5048B_RESOLUTION) * 60.0 * 360.0;
        break;
    case U_SOA:
        // second of arc
        angleConv = (angle / AS5048B_RESOLUTION) * 60.0 * 60.0 * 360.0;
        break;
    case U_GRAD:
        // grade
        angleConv = (angle / AS5048B_RESOLUTION) * 400.0;
        break;
    case U_MILNATO:
        // NATO MIL
        angleConv = (angle / AS5048B_RESOLUTION) * 6400.0;
        break;
    case U_MILSE:
        // Swedish MIL
        angleConv = (angle / AS5048B_RESOLUTION) * 6300.0;
        break;
    case U_MILRU:
        // Russian MIL
        angleConv = (angle / AS5048B_RESOLUTION) * 6000.0;
        break;
    default:
        // no conversion => raw angle
        angleConv = angle;
        break;
    }
    return angleConv;
}

double AS5048B::angleR(int unit, bool newVal) {
    double angleRaw;

    if (newVal) {
        if (_clockWise) {
            angleRaw = (double)(0b11111111111111 - AS5048B::readReg16(AS5048B_ANGLMSB_REG));
        }
        else {
            angleRaw = (double)AS5048B::readReg16(AS5048B_ANGLMSB_REG);
        }
        _lastAngleRaw = angleRaw;
    }
    else {
        angleRaw = _lastAngleRaw;
    }

    return AS5048B::convertAngle(unit, angleRaw);
}

void AS5048B::toggleDebug(void) {
    _debugFlag = !_debugFlag;
}

void AS5048B::setClockWise(bool cw) {
    _clockWise = cw;
    _lastAngleRaw = 0.0;
    AS5048B::resetMovingAvgExp();
}

void AS5048B::progRegister(uint8_t regVal) {
    AS5048B::writeReg(AS5048B_PROG_REG, regVal);
}

void AS5048B::doProg(void) {
    // enable special programming mode
    AS5048B::progRegister(0xFD);
    usleep(10000);

    // set the burn bit: enables automatic programming procedure
    AS5048B::progRegister(0x08);
    usleep(10000);

    // disable special programming mode
    AS5048B::progRegister(0x00);
    usleep(10000);

    return;
}

void AS5048B::doProgZero(void) {
    // this will burn the zero position OTP register like described in the datasheet
    // enable programming mode
    AS5048B::progRegister(0x01);
    usleep(10);

    // set the burn bit: enables automatic programming procedure
    AS5048B::progRegister(0x08);
    usleep(10);

    // read angle information (equals to 0)
    AS5048B::readReg16(AS5048B_ANGLMSB_REG);
    usleep(10);

    // enable verification
    AS5048B::progRegister(0x40);
    usleep(10);

    // read angle information (equals to 0)
    AS5048B::readReg16(AS5048B_ANGLMSB_REG);
    usleep(10);
}

void AS5048B::addressRegW(uint8_t regVal) {
    // write the new chip address to the register
    AS5048B::writeReg(AS5048B_ADDR_REG, regVal);
    // update our chip address with our 5 programmable bits
    // the MSB is internally inverted, so we flip the leftmost bit
    _chipAddress = ((regVal << 2) | (_chipAddress & 0b11)) ^ (1 << 6);
}

uint8_t AS5048B::addressRegR(void) {
    return AS5048B::readReg8(AS5048B_ADDR_REG);
}

void AS5048B::setZeroReg(void) {
    AS5048B::zeroRegW((uint16_t)0x00); // Issue closed by @MechatronicsWorkman and @oilXander. The last sequence avoids
    // any offset for the new Zero position
    uint16_t newZero = AS5048B::readReg16(AS5048B_ANGLMSB_REG);
    AS5048B::zeroRegW(newZero);
}

void AS5048B::zeroRegW(uint16_t regVal) {
    AS5048B::writeReg(AS5048B_ZEROMSB_REG, (uint8_t)(regVal >> 6));
    AS5048B::writeReg(AS5048B_ZEROLSB_REG, (uint8_t)(regVal & 0x3F));
}

uint16_t AS5048B::zeroRegR(void) {
    return AS5048B::readReg16(AS5048B_ZEROMSB_REG);
}

uint16_t AS5048B::magnitudeR(void) {
    return AS5048B::readReg16(AS5048B_MAGNMSB_REG);
}

uint16_t AS5048B::angleRegR(void) {
    return AS5048B::readReg16(AS5048B_ANGLMSB_REG);
}

uint8_t AS5048B::getAutoGain(void) {
    return AS5048B::readReg8(AS5048B_GAIN_REG);
}

uint8_t AS5048B::getDiagReg(void) {
    return AS5048B::readReg8(AS5048B_DIAG_REG);
}

// sine and cosine calculation on angles in radian
void AS5048B::updateMovingAvgExp(void) {
    double angle = AS5048B::angleR(U_RAD, true);

    if (_movingAvgCountLoop < EXP_MOVAVG_LOOP)
    {
        _movingAvgExpSin += sin(angle);
        _movingAvgExpCos += cos(angle);
        if (_movingAvgCountLoop == (EXP_MOVAVG_LOOP - 1)) {
            _movingAvgExpSin = _movingAvgExpSin / EXP_MOVAVG_LOOP;
            _movingAvgExpCos = _movingAvgExpCos / EXP_MOVAVG_LOOP;
        }
        _movingAvgCountLoop++;
    }
    else
    {
        double movavgexpsin = _movingAvgExpSin + _movingAvgExpAlpha * (sin(angle) - _movingAvgExpSin);
        double movavgexpcos = _movingAvgExpCos + _movingAvgExpAlpha * (cos(angle) - _movingAvgExpCos);
        _movingAvgExpSin = movavgexpsin;
        _movingAvgExpCos = movavgexpcos;
        _movingAvgExpAngle = getExpAvgRawAngle();
    }
}

double AS5048B::getMovingAvgExp(int unit) {
    return AS5048B::convertAngle(unit, _movingAvgExpAngle);
}

void AS5048B::resetMovingAvgExp(void) {
    _movingAvgExpAngle = 0.0;
    _movingAvgCountLoop = 0;
    _movingAvgExpAlpha = 2.0 / (EXP_MOVAVG_N + 1.0);
}

double AS5048B::getExpAvgRawAngle(void) {
    double angle;
    double twopi = 2 * M_PI;

    if (_movingAvgExpSin < 0.0) {
        angle = twopi - acos(_movingAvgExpCos);
    }
    else {
        angle = acos(_movingAvgExpCos);
    }
    angle = (angle / twopi) * AS5048B_RESOLUTION;

    return angle;
}

void AS5048B::printDebug(void) {
    return;
}

