
#ifndef _ICM_HPP_
#define _ICM_HPP_

#include <stdint.h>
#include "esp_err.h"
#include "sdkconfig.h"

#include "SPIbus.hpp" 

#include "icm/types.hpp"

/*! ICM Driver namespace */
namespace icm20601
{
class ICM;
}

/*! Easy alias for ICM class */
typedef icm20601::ICM ICM_t;

namespace icm20601
{
/*! Motion Processing Unit */
class ICM
{
 public:
    //! \name Constructors / Destructor
    //! \{
    ICM();
    explicit ICM(icm_bus_t& bus);
    ICM(icm_bus_t& bus, icm_addr_handle_t addr);
    ~ICM();
    //! \}
    //! \name Basic
    //! \{
    ICM& setBus(icm_bus_t& bus);
    ICM& setAddr(icm_addr_handle_t addr);
    icm_bus_t& getBus();
    icm_addr_handle_t getAddr();
    esp_err_t lastError();
    //! \}
    //! \name Setup
    //! \{
    esp_err_t initialize();
    esp_err_t reset();
    esp_err_t setSleep(bool enable);
    esp_err_t testConnection();
    esp_err_t selfTest(selftest_t* result);
    esp_err_t resetSignalPath();
    uint8_t whoAmI();
    bool getSleep();
    //! \}
    //! \name Main configurations
    //! \{
    esp_err_t setSampleRate(uint16_t rate);
    esp_err_t setClockSource(clock_src_t clockSrc);
    esp_err_t setDigitalLowPassFilter(dlpf_t dlpf);
    uint16_t getSampleRate();
    clock_src_t getClockSource();
    dlpf_t getDigitalLowPassFilter();
    //! \}
    //! \name Power management
    //! \{
    esp_err_t setLowPowerAccelMode(bool enable);
    esp_err_t setLowPowerAccelRate(lp_accel_rate_t rate);
    lp_accel_rate_t getLowPowerAccelRate();
    bool getLowPowerAccelMode();
    esp_err_t setStandbyMode(stby_en_t mask);
    stby_en_t getStandbyMode();
    //! \}
    //! \name Full-Scale Range
    //! \{
    esp_err_t setGyroFullScale(gyro_fs_t fsr);
    esp_err_t setAccelFullScale(accel_fs_t fsr);
    gyro_fs_t getGyroFullScale();
    accel_fs_t getAccelFullScale();
    //! \}
    //! \name Offset / Bias
    //! \{
    esp_err_t setGyroOffset(raw_axes_t bias);
    esp_err_t setAccelOffset(raw_axes_t bias);
    raw_axes_t getGyroOffset();
    raw_axes_t getAccelOffset();
    esp_err_t computeOffsets(raw_axes_t* accel, raw_axes_t* gyro);
    //! \}
    //! \name Interrupt
    //! \{
    esp_err_t setInterruptConfig(int_config_t config);
    esp_err_t setInterruptEnabled(int_en_t mask);
    int_stat_t getInterruptStatus();
    int_config_t getInterruptConfig();
    int_en_t getInterruptEnabled();
    //! \}
    //! \name FIFO
    //! \{
    esp_err_t setFIFOMode(fifo_mode_t mode);
    esp_err_t setFIFOConfig(fifo_config_t config);
    esp_err_t setFIFOEnabled(bool enable);
    esp_err_t resetFIFO();
    uint16_t getFIFOCount();
    esp_err_t readFIFO(size_t length, uint8_t* data);
    esp_err_t writeFIFO(size_t length, const uint8_t* data);
    fifo_mode_t getFIFOMode();
    fifo_config_t getFIFOConfig();
    bool getFIFOEnabled();
    //! \}
    //! \name Auxiliary I2C Master
    //! \{
    esp_err_t setAuxI2CConfig(const auxi2c_config_t& config);
    esp_err_t setAuxI2CEnabled(bool enable);
    esp_err_t setAuxI2CSlaveConfig(const auxi2c_slv_config_t& config);
    esp_err_t setAuxI2CSlaveEnabled(auxi2c_slv_t slave, bool enable);
    esp_err_t setAuxI2CBypass(bool enable);
    esp_err_t readAuxI2CRxData(size_t length, uint8_t* data, size_t skip = 0);
    esp_err_t restartAuxI2C();
    auxi2c_stat_t getAuxI2CStatus();
    auxi2c_config_t getAuxI2CConfig();
    auxi2c_slv_config_t getAuxI2CSlaveConfig(auxi2c_slv_t slave);
    bool getAuxI2CEnabled();
    bool getAuxI2CSlaveEnabled(auxi2c_slv_t slave);
    bool getAuxI2CBypass();
    esp_err_t auxI2CWriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    esp_err_t auxI2CReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data);
    //! \}
    //! \name Motion Detection Interrupt
    //! \{
    esp_err_t setMotionDetectConfig(mot_config_t& config);
    mot_config_t getMotionDetectConfig();
    esp_err_t setMotionFeatureEnabled(bool enable);
    bool getMotionFeatureEnabled();
    //! \}
    //! \name Miscellaneous
    //! \{
    esp_err_t setFsyncConfig(int_lvl_t level);
    esp_err_t setFsyncEnabled(bool enable);
    int_lvl_t getFsyncConfig();
    bool getFsyncEnabled();
    esp_err_t setFchoice(fchoice_t fchoice);
    fchoice_t getFchoice();
    //! \}
    //! \name Read / Write
    //! Functions to perform direct read or write operation(s) to registers.
    //! \{
    esp_err_t readBit(uint8_t regAddr, uint8_t bitNum, uint8_t* data);
    esp_err_t readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
    esp_err_t readByte(uint8_t regAddr, uint8_t* data);
    esp_err_t readBytes(uint8_t regAddr, size_t length, uint8_t* data);
    esp_err_t writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
    esp_err_t writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    esp_err_t writeByte(uint8_t regAddr, uint8_t data);
    esp_err_t writeBytes(uint8_t regAddr, size_t length, const uint8_t* data);
    esp_err_t registerDump(uint8_t start = 0x0, uint8_t end = 0x7F);
    //! \}
    //! \name Sensor readings
    //! \{
    esp_err_t acceleration(raw_axes_t* accel);
    esp_err_t acceleration(int16_t* x, int16_t* y, int16_t* z);
    esp_err_t rotation(raw_axes_t* gyro);
    esp_err_t rotation(int16_t* x, int16_t* y, int16_t* z);
    esp_err_t temperature(int16_t* temp);
    esp_err_t motion(raw_axes_t* accel, raw_axes_t* gyro);
    esp_err_t heading(raw_axes_t* mag);
    esp_err_t heading(int16_t* x, int16_t* y, int16_t* z);
    esp_err_t motion(raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag);
    esp_err_t sensors(raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp);
    esp_err_t sensors(sensors_t* sensors, size_t extsens_len = 0);
    //! \}

 protected:
    esp_err_t accelSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result);
    esp_err_t gyroSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result);
    esp_err_t getBiases(accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                        bool selftest);

    icm_bus_t* bus;         /*!< Communication bus pointer, I2C / SPI */
    icm_addr_handle_t addr; /*!< I2C address / SPI device handle */
    uint8_t buffer[16];     /*!< Commom buffer for temporary data */
    esp_err_t err;          /*!< Holds last error code */
};

}  // namespace icm20601

// ==============
// Inline methods
// ==============
namespace icm20601
{
/*! Default Constructor. */
inline ICM::ICM() : ICM(ICM_DEFAULT_BUS){};
/**
 * @brief Contruct a ICM in the given communication bus.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 */
inline ICM::ICM(icm_bus_t& bus) : ICM(bus, ICM_DEFAULT_ADDR_HANDLE) {}
/**
 * @brief Construct a ICM in the given communication bus and address.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
inline ICM::ICM(icm_bus_t& bus, icm_addr_handle_t addr) : bus{&bus}, addr{addr}, buffer{0}, err{ESP_OK} {}
/** Default Destructor, does nothing. */
inline ICM::~ICM() = default;
/**
 * @brief Set communication bus.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 */
inline ICM& ICM::setBus(icm_bus_t& bus)
{
    this->bus = &bus;
    return *this;
}
/**
 * @brief Return communication bus object.
 */
inline icm_bus_t& ICM::getBus()
{
    return *bus;
}
/**
 * @brief Set I2C address or SPI device handle.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
inline ICM& ICM::setAddr(icm_addr_handle_t addr)
{
    this->addr = addr;
    return *this;
}
/**
 * @brief Return I2C address or SPI device handle.
 */
inline icm_addr_handle_t ICM::getAddr()
{
    return addr;
}
/*! Return last error code. */
inline esp_err_t ICM::lastError()
{
    return err;
}
/*! Read a single bit from a register*/
inline esp_err_t ICM::readBit(uint8_t regAddr, uint8_t bitNum, uint8_t* data)
{
    return err = bus->readBit(addr, regAddr, bitNum, data);
}
/*! Read a range of bits from a register */
inline esp_err_t ICM::readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data)
{
    return err = bus->readBits(addr, regAddr, bitStart, length, data);
}
/*! Read a single register */
inline esp_err_t ICM::readByte(uint8_t regAddr, uint8_t* data)
{
    return err = bus->readByte(addr, regAddr, data);
}
/*! Read data from sequence of registers */
inline esp_err_t ICM::readBytes(uint8_t regAddr, size_t length, uint8_t* data)
{
    return err = bus->readBytes(addr, regAddr, length, data);
}
/*! Write a single bit to a register */
inline esp_err_t ICM::writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    return err = bus->writeBit(addr, regAddr, bitNum, data);
}
/*! Write a range of bits to a register */
inline esp_err_t ICM::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    return err = bus->writeBits(addr, regAddr, bitStart, length, data);
}
/*! Write a value to a register */
inline esp_err_t ICM::writeByte(uint8_t regAddr, uint8_t data)
{
    return err = bus->writeByte(addr, regAddr, data);
}
/*! Write a sequence to data to a sequence of registers */
inline esp_err_t ICM::writeBytes(uint8_t regAddr, size_t length, const uint8_t* data)
{
    return err = bus->writeBytes(addr, regAddr, length, data);
}

}  // namespace icm20601

#endif /* end of include guard: _ICM_HPP_ */