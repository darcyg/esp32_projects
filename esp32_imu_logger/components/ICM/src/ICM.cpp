#include "ICM.hpp"
#include <math.h>
#include <string.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "icm/math.hpp"
#include "icm/registers.hpp"
#include "icm/types.hpp"
#include "sdkconfig.h"

static const char* TAG = "ICM20601";

#include "icm/log.hpp"

/*! ICM 20601 Driver namespace */
namespace icm20601
{
/**
 * @brief Initialize ICM device and set basic configurations.
 * @details
 *  Init configuration:
 *  - Accel FSR: 4G
 *  - Gyro FSR: 500DPS
 *  - Sample rate: 100Hz
 *  - DLPF: 42Hz
 *  - INT pin: disabled
 *  - FIFO: disabled 
 *  - Clock source: gyro PLL \n
 *  For MPU9150 and MPU9250:
 *  - Aux I2C Master: enabled, clock: 400KHz
 *  - Compass: enabled on Aux I2C's Slave 0 and Slave 1
 *
 * @note
 *  - A soft reset is performed first, which takes 100-200ms.
 *  - When using SPI, the primary I2C Slave module is disabled right away.
 * */
esp_err_t ICM::initialize()
{
    // reset device (wait a little to clear all registers)
    if (ICM_ERR_CHECK(reset())) return err;
    // wake-up the device (power on-reset state is asleep for some models)
    if (ICM_ERR_CHECK(setSleep(false))) return err;
        // disable ICM's I2C slave module when using SPI
    if (ICM_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_I2C_IF_DIS_BIT, 1))) return err;
    // set clock source to gyro PLL which is better than internal clock
    if (ICM_ERR_CHECK(setClockSource(CLOCK_PLL))) return err;

    // MPU6500 / MPU9250 share 4kB of memory between the DMP and the FIFO. Since the
    // first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
    // if (ICM_ERR_CHECK(writeBits(regs::ACCEL_CONFIG2, regs::ACONFIG2_FIFO_SIZE_BIT, regs::ACONFIG2_FIFO_SIZE_LENGTH,
    //                            FIFO_SIZE_1K))) {
    //    return err;
    // }
    // set Full Scale range
    if (ICM_ERR_CHECK(setGyroFullScale(GYRO_FS_500DPS))) return err;
    if (ICM_ERR_CHECK(setAccelFullScale(ACCEL_FS_4G))) return err;
    // set Digital Low Pass Filter to get smoother data
    if (ICM_ERR_CHECK(setDigitalLowPassFilter(DLPF_42HZ))) return err;

    // set sample rate to 100Hz
    if (ICM_ERR_CHECK(setSampleRate(100))) return err;
    ICM_LOGI("Initialization complete");
    return err;
}

/**
 * @brief Reset internal registers and restore to default start-up state.
 * @note
 *  - This function delays 100ms when using I2C and 200ms when using SPI.
 *  - It does not initialize the ICM again, just call initialize() instead.
 * */
esp_err_t ICM::reset()
{
    if (ICM_ERR_CHECK(writeBit(regs::PWR_MGMT1, regs::PWR1_DEVICE_RESET_BIT, 1))) return err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (ICM_ERR_CHECK(resetSignalPath())) {
        return err;
    }
    ICM_LOGI("Reset!");
    return err;
}

/**
 * @brief Enable / disable sleep mode
 * @param enable enable value
 * */
esp_err_t ICM::setSleep(bool enable)
{
    return ICM_ERR_CHECK(writeBit(regs::PWR_MGMT1, regs::PWR1_SLEEP_BIT, (uint8_t) enable));
}

/**
 * @brief Get current sleep state.
 * @return
 *  - `true`: sleep enabled.
 *  - `false`: sleep disabled.
 */
bool ICM::getSleep()
{
    ICM_ERR_CHECK(readBit(regs::PWR_MGMT1, regs::PWR1_SLEEP_BIT, buffer));
    return buffer[0];
}

/**
 * @brief Test connection with ICM.
 * @details It reads the WHO_AM_IM register and check its value against the correct chip model.
 * @return
 *  - `ESP_OK`: The ICM is connected and matchs the model.
 *  - `ESP_ERR_NOT_FOUND`: A device is connect, but does not match the chip selected in _menuconfig_.
 *  - May return other communication bus errors. e.g: `ESP_FAIL`, `ESP_ERR_TIMEOUT`.
 * */
esp_err_t ICM::testConnection()
{
    const uint8_t wai = whoAmI();
    if (ICM_ERR_CHECK(lastError())) return err;
    // return (wai == 0x71) ? ESP_OK : ESP_ERR_NOT_FOUND;
    return (wai == 0xAC) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/**
 * @brief Returns the value from WHO_AM_I register.
 */
uint8_t ICM::whoAmI()
{
    ICM_ERR_CHECK(readByte(regs::WHO_AM_I, buffer));
    return buffer[0];
}

/**
 * @brief Set sample rate of data output.
 *
 * Sample rate controls sensor data output rate and FIFO sample rate.
 * This is the update rate of sensor register. \n
 * Formula: Sample Rate = Internal Output Rate / (1 + SMPLRT_DIV)
 *
 * @param rate 4Hz ~ 1KHz
 *  - For sample rate 8KHz: set digital low pass filter to DLPF_256HZ_NOLPF.
 *  - For sample rate 32KHZ [MPU6500 / MPU9250]: set fchoice to FCHOICE_0, see setFchoice().
 *
 * @note
 *  For MPU9150 & MPU9250:
 *   - When using compass, this function alters Aux I2C Master `sample_delay` property
 *     to adjust the compass sample rate. (also, `wait_for_es` property to adjust interrupt).
 *   - If sample rate lesser than 100 Hz, data-ready interrupt will wait for compass data.
 *   - If sample rate greater than 100 Hz, data-ready interrupt will not be delayed by the compass.
 * */
esp_err_t ICM::setSampleRate(uint16_t rate)
{
    // Check value range
    if (rate < 4) {
        ICM_LOGWMSG(msgs::INVALID_SAMPLE_RATE, " %d, minimum rate is 4", rate);
        rate = 4;
    }
    else if (rate > 1000) {
        ICM_LOGWMSG(msgs::INVALID_SAMPLE_RATE, " %d, maximum rate is 1000", rate);
        rate = 1000;
    }


    fchoice_t fchoice = getFchoice();
    if (ICM_ERR_CHECK(lastError())) return err;
    if (fchoice != FCHOICE_3) {
        ICM_LOGWMSG(msgs::INVALID_STATE, ", sample rate divider is not effective when Fchoice != 3");
    }
    // Check dlpf configuration
    dlpf_t dlpf = getDigitalLowPassFilter();
    if (ICM_ERR_CHECK(lastError())) return err;
    if (dlpf == 0 || dlpf == 7)
        ICM_LOGWMSG(msgs::INVALID_STATE, ", sample rate divider is not effective when DLPF is (0 or 7)");

    constexpr uint16_t internalSampleRate = 1000;
    uint16_t divider                      = internalSampleRate / rate - 1;
    // Check for rate match
    uint16_t finalRate = (internalSampleRate / (1 + divider));
    if (finalRate != rate) {
        ICM_LOGW("Sample rate constrained to %d Hz", finalRate);
    }
    else {
        ICM_LOGI("Sample rate set to %d Hz", finalRate);
    }
    // Write divider to register
    if (ICM_ERR_CHECK(writeByte(regs::SMPLRT_DIV, (uint8_t) divider))) return err;

    return err;
}

/**
 * @brief Retrieve sample rate divider and calculate the actual rate.
 */
uint16_t ICM::getSampleRate()
{
    fchoice_t fchoice = getFchoice();
    ICM_ERR_CHECK(lastError());
    if (fchoice != FCHOICE_3) return SAMPLE_RATE_MAX;
    constexpr uint16_t sampleRateMax_nolpf = 8000;
    dlpf_t dlpf                            = getDigitalLowPassFilter();
    ICM_ERR_CHECK(lastError());
    if (dlpf == 0 || dlpf == 7) return sampleRateMax_nolpf;

    constexpr uint16_t internalSampleRate = 1000;
    ICM_ERR_CHECK(readByte(regs::SMPLRT_DIV, buffer));
    uint16_t rate = internalSampleRate / (1 + buffer[0]);
    return rate;
}

/**
 * @brief Select clock source.
 * @note The gyro PLL is better than internal clock.
 * @param clockSrc clock source
 */
esp_err_t ICM::setClockSource(clock_src_t clockSrc)
{
    return ICM_ERR_CHECK(writeBits(regs::PWR_MGMT1, regs::PWR1_CLKSEL_BIT, regs::PWR1_CLKSEL_LENGTH, clockSrc));
}

/**
 * @brief Return clock source.
 */
clock_src_t ICM::getClockSource()
{
    ICM_ERR_CHECK(readBits(regs::PWR_MGMT1, regs::PWR1_CLKSEL_BIT, regs::PWR1_CLKSEL_LENGTH, buffer));
    return (clock_src_t) buffer[0];
}

/**
 * @brief Configures Digital Low Pass Filter (DLPF) setting for both the gyroscope and accelerometer.
 * @param dlpf digital low-pass filter value
 */
esp_err_t ICM::setDigitalLowPassFilter(dlpf_t dlpf)
{
    if (ICM_ERR_CHECK(writeBits(regs::CONFIG, regs::CONFIG_DLPF_CFG_BIT, regs::CONFIG_DLPF_CFG_LENGTH, dlpf))) {
        return err;
    }
    ICM_ERR_CHECK(
        writeBits(regs::ACCEL_CONFIG2, regs::ACONFIG2_A_DLPF_CFG_BIT, regs::ACONFIG2_A_DLPF_CFG_LENGTH, dlpf));
    return err;
}

/**
 * @brief Return Digital Low Pass Filter configuration
 */
dlpf_t ICM::getDigitalLowPassFilter()
{
    ICM_ERR_CHECK(readBits(regs::CONFIG, regs::CONFIG_DLPF_CFG_BIT, regs::CONFIG_DLPF_CFG_LENGTH, buffer));
    return (dlpf_t) buffer[0];
}

/**
 * @brief Reset sensors signal path.
 *
 * Reset all gyro digital signal path, accel digital signal path, and temp
 * digital signal path. This also clears all the sensor registers.
 *
 * @note This function delays 100 ms, needed for reset to complete.
 * */
esp_err_t ICM::resetSignalPath()
{
    if (ICM_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_SIG_COND_RESET_BIT, 1))) return err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return err;
}

/**
 * @brief Enter Low Power Accelerometer mode.
 *
 * In low-power accel mode, the chip goes to sleep and only wakes up to sample
 * the accelerometer at a certain frequency.
 * See setLowPowerAccelRate() to set the frequency.
 *
 * @param enable value
 *  + This function does the following to enable:
 *   - Set CYCLE bit to 1
 *   - Set SLEEP bit to 0
 *   - Set TEMP_DIS bit to 1
 *   - Set STBY_XG, STBY_YG, STBY_ZG bits to 1
 *   - Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *   - Set FCHOICE to 0 (ACCEL_FCHOICE_B bit to 1) [MPU6500 / MPU9250 only]
 *   - Disable Auxiliary I2C Master I/F
 *
 *  + This function does the following to disable:
 *   - Set CYCLE bit to 0
 *   - Set TEMP_DIS bit to 0
 *   - Set STBY_XG, STBY_YG, STBY_ZG bits to 0
 *   - Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *   - Set FCHOICE to 3 (ACCEL_FCHOICE_B bit to 0) [MPU6500 / MPU9250 only]
 *   - Enable Auxiliary I2C Master I/F
 * */
esp_err_t ICM::setLowPowerAccelMode(bool enable)
{
    fchoice_t fchoice = enable ? FCHOICE_0 : FCHOICE_3;
    if (ICM_ERR_CHECK(setFchoice(fchoice))) return err;
    ICM_LOGVMSG(msgs::EMPTY, "Fchoice set to %d", fchoice);
    // read PWR_MGMT1 and PWR_MGMT2 at once
    if (ICM_ERR_CHECK(readBytes(regs::PWR_MGMT1, 2, buffer))) return err;
    if (enable) {
        // set CYCLE bit to 1 and SLEEP bit to 0 and TEMP_DIS bit to 1
        buffer[0] |= 1 << regs::PWR1_CYCLE_BIT;
        buffer[0] &= ~(1 << regs::PWR1_SLEEP_BIT);
        buffer[0] |= 1 << regs::PWR1_TEMP_DIS_BIT;
        // set STBY_XG, STBY_YG, STBY_ZG bits to 1
        buffer[1] |= regs::PWR2_STBY_XYZG_BITS;
    }
    else {  // disable
        // set CYCLE bit to 0 and TEMP_DIS bit to 0
        buffer[0] &= ~(1 << regs::PWR1_CYCLE_BIT);
        buffer[0] &= ~(1 << regs::PWR1_TEMP_DIS_BIT);
        // set STBY_XG, STBY_YG, STBY_ZG bits to 0
        buffer[1] &= ~(regs::PWR2_STBY_XYZG_BITS);
    }
    // set STBY_XA, STBY_YA, STBY_ZA bits to 0
    buffer[1] &= ~(regs::PWR2_STBY_XYZA_BITS);
    // write back PWR_MGMT1 and PWR_MGMT2 at once
    if (ICM_ERR_CHECK(writeBytes(regs::PWR_MGMT1, 2, buffer))) return err;
    // disable Auxiliary I2C Master I/F in case it was active
    if (ICM_ERR_CHECK(setAuxI2CEnabled(!enable))) return err;
    return err;
}

/**
 * @brief Return Low Power Accelerometer state.
 *
 * Condition to return true:
 *  - CYCLE bit is 1
 *  - SLEEP bit is 0
 *  - TEMP_DIS bit is 1
 *  - STBY_XG, STBY_YG, STBY_ZG bits are 1
 *  - STBY_XA, STBY_YA, STBY_ZA bits are 0
 *  - FCHOICE is 0 (ACCEL_FCHOICE_B bit is 1) [MPU6500 / MPU9250 only]
 *
 * */
bool ICM::getLowPowerAccelMode()
{
    // check FCHOICE
    fchoice_t fchoice = getFchoice();
    ICM_ERR_CHECK(lastError());
    if (fchoice != FCHOICE_0) {
        return false;
    }
    // read PWR_MGMT1 and PWR_MGMT2 at once
    ICM_ERR_CHECK(readBytes(regs::PWR_MGMT1, 2, buffer));
    // define configuration bits
    constexpr uint8_t LPACCEL_CONFIG_BITMASK[2] = {
        (1 << regs::PWR1_SLEEP_BIT) | (1 << regs::PWR1_CYCLE_BIT) | (1 << regs::PWR1_TEMP_DIS_BIT),
        regs::PWR2_STBY_XYZA_BITS | regs::PWR2_STBY_XYZG_BITS};
    constexpr uint8_t LPACCEL_ENABLED_VALUE[2] = {(1 << regs::PWR1_CYCLE_BIT) | (1 << regs::PWR1_TEMP_DIS_BIT),
                                                  regs::PWR2_STBY_XYZG_BITS};
    // get just the configuration bits
    buffer[0] &= LPACCEL_CONFIG_BITMASK[0];
    buffer[1] &= LPACCEL_CONFIG_BITMASK[1];
    // check pattern
    return buffer[0] == LPACCEL_ENABLED_VALUE[0] && buffer[1] == LPACCEL_ENABLED_VALUE[1];
}

/**
 * @brief Set Low Power Accelerometer frequency of wake-up.
 * */
esp_err_t ICM::setLowPowerAccelRate(lp_accel_rate_t rate)
{
    return ICM_ERR_CHECK(writeBits(regs::PWR_MGMT2, regs::PWR2_LP_WAKE_CTRL_BIT, regs::PWR2_LP_WAKE_CTRL_LENGTH, rate));
}

/**
 * @brief Get Low Power Accelerometer frequency of wake-up.
 */
lp_accel_rate_t ICM::getLowPowerAccelRate()
{
        ICM_ERR_CHECK(readBits(regs::LP_ACCEL_ODR, regs::LPA_ODR_CLKSEL_BIT, regs::LPA_ODR_CLKSEL_LENGTH, buffer));
    return (lp_accel_rate_t) buffer[0];
}

/**
 * @brief Enable/disable Motion modules (Motion detect, Zero-motion, Free-Fall).
 *
 * @attention
 *  The configurations must've already been set with setMotionDetectConfig() before
 *  enabling the module!
 * @note
 *  - Call getMotionDetectStatus() to find out which axis generated motion interrupt. [ICM6000, MPU6050, MPU9150].
 *  - It is recommended to set the Motion Interrupt to propagate to the INT pin. To do that, use setInterruptEnabled().
 * @param enable
 *  - On _true_, this function modifies the DLPF, put gyro and temperature in standby,
 *    and disable Auxiliary I2C Master I/F.
 *  - On _false_, this function sets DLPF to 42Hz and enables Auxiliary I2C master I/F.
 * */
esp_err_t ICM::setMotionFeatureEnabled(bool enable)
{
    /* enabling */
    if (enable) {
        constexpr dlpf_t kDLPF = DLPF_188HZ;
        if (ICM_ERR_CHECK(setDigitalLowPassFilter(kDLPF))) return err;

        if (ICM_ERR_CHECK(
                writeByte(regs::ACCEL_INTEL_CTRL, (1 << regs::ACCEL_INTEL_EN_BIT) | (1 << regs::ACCEL_INTEL_MODE_BIT))))
            return err;
        /* disabling */
    }
    else {
        if (ICM_ERR_CHECK(writeBits(regs::ACCEL_INTEL_CTRL, regs::ACCEL_INTEL_EN_BIT, 2, 0x0))) {
            return err;
        }
        constexpr dlpf_t kDLPF = DLPF_42HZ;
        if (ICM_ERR_CHECK(setDigitalLowPassFilter(kDLPF))) return err;
    }
    // disable Auxiliary I2C Master I/F in case it was active
    if (ICM_ERR_CHECK(setAuxI2CEnabled(!enable))) return err;
    return err;
}

/**
 * @brief Return true if a Motion Dectection module is enabled.
 */
bool ICM::getMotionFeatureEnabled()
{
    uint8_t data;
    ICM_ERR_CHECK(readByte(regs::ACCEL_INTEL_CTRL, &data));
    constexpr uint8_t kAccelIntel = (1 << regs::ACCEL_INTEL_EN_BIT) | (1 << regs::ACCEL_INTEL_MODE_BIT);
    if ((data & kAccelIntel) != kAccelIntel) return false;
    constexpr dlpf_t kDLPF = DLPF_188HZ;
    dlpf_t dlpf = getDigitalLowPassFilter();
    ICM_ERR_CHECK(lastError());
    if (dlpf != kDLPF) return false;
    return true;
}

/**
 * @brief Configure Motion-Detect or Wake-on-motion feature.
 *
 * The behaviour of this feature is very different between the MPU6050 (MPU9150) and the
 * MPU6500 (MPU9250). Each chip's version of this feature is explained below.
 *
 * [MPU6050, MPU6000, MPU9150]:
 * Accelerometer measurements are passed through a configurable digital high pass filter (DHPF)
 * in order to eliminate bias due to gravity. A qualifying motion sample is one where the high
 * passed sample from any axis has an absolute value exceeding a user-programmable threshold. A
 * counter increments for each qualifying sample, and decrements for each non-qualifying sample.
 * Once the counter reaches a user-programmable counter threshold, a motion interrupt is triggered.
 * The axis and polarity which caused the interrupt to be triggered is flagged in the
 * MOT_DETECT_STATUS register.
 *
 * [MPU6500, MPU9250]:
 * Unlike the MPU6050 version, the hardware does not "lock in" a reference sample.
 * The hardware monitors the accel data and detects any large change over a short period of time.
 * A qualifying motion sample is one where the high passed sample from any axis has
 * an absolute value exceeding the threshold.
 * The hardware motion threshold can be between 4mg and 1020mg in 4mg increments.
 *
 * @note
 * It is possible to enable **wake-on-motion** mode by doing the following:
 *  1. Enter Low Power Accelerometer mode with setLowPowerAccelMode();
 *  2. Select the wake-up rate with setLowPowerAccelRate();
 *  3. Configure motion-detect interrupt with setMotionDetectConfig();
 *  4. Enable the motion detection module with setMotionFeatureEnabled();
 * */
esp_err_t ICM::setMotionDetectConfig(mot_config_t& config)
{
    return ICM_ERR_CHECK(writeByte(regs::MOTION_THR, config.threshold));
}

/**
 * @brief Return Motion Detection Configuration.
 */
mot_config_t ICM::getMotionDetectConfig()
{
    mot_config_t config{};
    ICM_ERR_CHECK(readByte(regs::MOTION_THR, &config.threshold));
    return config;
}

/**
 * @brief Configure sensors' standby mode.
 * */
esp_err_t ICM::setStandbyMode(stby_en_t mask)
{
    const uint8_t kPwr1StbyBits = mask >> 6;
    if (ICM_ERR_CHECK(writeBits(regs::PWR_MGMT1, regs::PWR1_GYRO_STANDBY_BIT, 2, kPwr1StbyBits))) {
        return err;
    }
    return ICM_ERR_CHECK(writeBits(regs::PWR_MGMT2, regs::PWR2_STBY_XA_BIT, 6, mask));
}

/**
 * @brief Return Standby configuration.
 * */
stby_en_t ICM::getStandbyMode()
{
    ICM_ERR_CHECK(readBytes(regs::PWR_MGMT1, 2, buffer));
    constexpr uint8_t kStbyTempAndGyroPLLBits = STBY_EN_TEMP | STBY_EN_LOWPWR_GYRO_PLL_ON;
    stby_en_t mask                            = buffer[0] << 3 & kStbyTempAndGyroPLLBits;
    constexpr uint8_t kStbyAccelAndGyroBits   = STBY_EN_ACCEL | STBY_EN_GYRO;
    mask |= buffer[1] & kStbyAccelAndGyroBits;
    return mask;
}

/**
 * @brief Select FCHOICE.
 *
 * Dev note: FCHOICE is the inverted value of FCHOICE_B (e.g. FCHOICE=2b’00 is same as FCHOICE_B=2b’11).
 * Reset value is FCHOICE_3
 * */
esp_err_t ICM::setFchoice(fchoice_t fchoice)
{
    buffer[0] = (~(fchoice) &0x3);  // invert to fchoice_b
    if (ICM_ERR_CHECK(
            writeBits(regs::GYRO_CONFIG, regs::GCONFIG_FCHOICE_B, regs::GCONFIG_FCHOICE_B_LENGTH, buffer[0]))) {
        return err;
    }
    return ICM_ERR_CHECK(writeBit(regs::ACCEL_CONFIG2, regs::ACONFIG2_ACCEL_FCHOICE_B_BIT, (buffer[0] == 0) ? 0 : 1));
}

/**
 * @brief Return FCHOICE.
 */
fchoice_t ICM::getFchoice()
{
    ICM_ERR_CHECK(readBits(regs::GYRO_CONFIG, regs::GCONFIG_FCHOICE_B, regs::GCONFIG_FCHOICE_B_LENGTH, buffer));
    return (fchoice_t)(~(buffer[0]) & 0x3);
}

/**
 * @brief Select Gyroscope Full-scale range.
 * */
esp_err_t ICM::setGyroFullScale(gyro_fs_t fsr)
{
    return ICM_ERR_CHECK(writeBits(regs::GYRO_CONFIG, regs::GCONFIG_FS_SEL_BIT, regs::GCONFIG_FS_SEL_LENGTH, fsr));
}

/**
 * @brief Return Gyroscope Full-scale range.
 */
gyro_fs_t ICM::getGyroFullScale()
{
    ICM_ERR_CHECK(readBits(regs::GYRO_CONFIG, regs::GCONFIG_FS_SEL_BIT, regs::GCONFIG_FS_SEL_LENGTH, buffer));
    return (gyro_fs_t) buffer[0];
}

/**
 * @brief Select Accelerometer Full-scale range.
 * */
esp_err_t ICM::setAccelFullScale(accel_fs_t fsr)
{
    return ICM_ERR_CHECK(writeBits(regs::ACCEL_CONFIG, regs::ACONFIG_FS_SEL_BIT, regs::ACONFIG_FS_SEL_LENGTH, fsr));
}

/**
 * @brief Return Accelerometer Full-scale range.
 */
accel_fs_t ICM::getAccelFullScale()
{
    ICM_ERR_CHECK(readBits(regs::ACCEL_CONFIG, regs::ACONFIG_FS_SEL_BIT, regs::ACONFIG_FS_SEL_LENGTH, buffer));
    return (accel_fs_t) buffer[0];
}

/**
 * @brief Push biases to the gyro offset registers.
 *
 * This function expects biases relative to the current sensor output, and
 * these biases will be added to the factory-supplied values.
 *
 * Note: Bias inputs are LSB in +-1000dps format.
 * */
esp_err_t ICM::setGyroOffset(raw_axes_t bias)
{
    buffer[0] = (uint8_t)(bias.x >> 8);
    buffer[1] = (uint8_t)(bias.x);
    buffer[2] = (uint8_t)(bias.y >> 8);
    buffer[3] = (uint8_t)(bias.y);
    buffer[4] = (uint8_t)(bias.z >> 8);
    buffer[5] = (uint8_t)(bias.z);
    return ICM_ERR_CHECK(writeBytes(regs::XG_OFFSET_H, 6, buffer));
}

/**
 * @brief Return biases from the gyro offset registers.
 *
 * Note: Bias output are LSB in +-1000dps format.
 * */
raw_axes_t ICM::getGyroOffset()
{
    ICM_ERR_CHECK(readBytes(regs::XG_OFFSET_H, 6, buffer));
    raw_axes_t bias;
    bias.x = (buffer[0] << 8) | buffer[1];
    bias.y = (buffer[2] << 8) | buffer[3];
    bias.z = (buffer[4] << 8) | buffer[5];
    return bias;
}

/**
 * @brief Push biases to the accel offset registers.
 *
 * This function expects biases relative to the current sensor output, and
 * these biases will be added to the factory-supplied values.
 *
 * Note: Bias inputs are LSB in +-16G format.
 * */
esp_err_t ICM::setAccelOffset(raw_axes_t bias)
{
    raw_axes_t facBias;
    // first, read OTP values of Accel factory trim
    if (ICM_ERR_CHECK(readBytes(regs::XA_OFFSET_H, 8, buffer))) return err;
    // note: buffer[2] and buffer[5], stay the same,
    //  they are read just to keep the burst reading
    facBias.x = (buffer[0] << 8) | buffer[1];
    facBias.y = (buffer[3] << 8) | buffer[4];
    facBias.z = (buffer[6] << 8) | buffer[7];
    // note: preserve bit 0 of factory value (for temperature compensation)
    facBias.x += (bias.x & ~1);
    facBias.y += (bias.y & ~1);
    facBias.z += (bias.z & ~1);

    buffer[0] = (uint8_t)(facBias.x >> 8);
    buffer[1] = (uint8_t)(facBias.x);
    buffer[3] = (uint8_t)(facBias.y >> 8);
    buffer[4] = (uint8_t)(facBias.y);
    buffer[6] = (uint8_t)(facBias.z >> 8);
    buffer[7] = (uint8_t)(facBias.z);
    return ICM_ERR_CHECK(writeBytes(regs::XA_OFFSET_H, 8, buffer));
    return err;
}

/**
 * @brief Return biases from accel offset registers.
 * This returns the biases with OTP values from factory trim added,
 * so returned values will be different than that ones set with setAccelOffset().
 *
 * Note: Bias output are LSB in +-16G format.
 * */
raw_axes_t ICM::getAccelOffset()
{
    raw_axes_t bias;

    ICM_ERR_CHECK(readBytes(regs::XA_OFFSET_H, 8, buffer));
    bias.x                        = (buffer[0] << 8) | buffer[1];
    bias.y                        = (buffer[3] << 8) | buffer[4];
    bias.z                        = (buffer[6] << 8) | buffer[7];

    return bias;
}

/**
 * @brief Compute Accelerometer and Gyroscope offsets.
 *
 * This takes about ~400ms to compute offsets.
 * When calculating the offsets the MPU must remain as horizontal as possible (0 degrees), facing
 * up. It is better to call computeOffsets() before any configuration is done (better right after
 * initialize()).
 *
 * Note: Gyro offset output are LSB in 1000DPS format.
 * Note: Accel offset output are LSB in 16G format.
 * */
esp_err_t ICM::computeOffsets(raw_axes_t* accel, raw_axes_t* gyro)
{
    constexpr accel_fs_t kAccelFS = ACCEL_FS_4G;     // most sensitive
    constexpr gyro_fs_t kGyroFS   = GYRO_FS_500DPS;  // most sensitive
    if (ICM_ERR_CHECK(getBiases(kAccelFS, kGyroFS, accel, gyro, false))) return err;
    // convert offsets to 16G and 1000DPS format and invert values
    for (int i = 0; i < 3; i++) {
        (*accel)[i] = -((*accel)[i] >> (types::ACCEL_FS_16G - kAccelFS));
        (*gyro)[i]  = -((*gyro)[i] >> (types::GYRO_FS_1000DPS - kGyroFS));
    }
    return err;
}

/**
 * @brief Read accelerometer raw data.
 * */
esp_err_t ICM::acceleration(raw_axes_t* accel)
{
    if (ICM_ERR_CHECK(readBytes(regs::ACCEL_XOUT_H, 6, buffer))) return err;
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];
    return err;
}

/**
 * @brief Read accelerometer raw data.
 * */
esp_err_t ICM::acceleration(int16_t* x, int16_t* y, int16_t* z)
{
    if (ICM_ERR_CHECK(readBytes(regs::ACCEL_XOUT_H, 6, buffer))) return err;
    *x = buffer[0] << 8 | buffer[1];
    *y = buffer[2] << 8 | buffer[3];
    *z = buffer[4] << 8 | buffer[5];
    return err;
}

/**
 * @brief Read gyroscope raw data.
 * */
esp_err_t ICM::rotation(raw_axes_t* gyro)
{
    if (ICM_ERR_CHECK(readBytes(regs::GYRO_XOUT_H, 6, buffer))) return err;
    gyro->x = buffer[0] << 8 | buffer[1];
    gyro->y = buffer[2] << 8 | buffer[3];
    gyro->z = buffer[4] << 8 | buffer[5];
    return err;
}

/**
 * @brief Read gyroscope raw data.
 * */
esp_err_t ICM::rotation(int16_t* x, int16_t* y, int16_t* z)
{
    if (ICM_ERR_CHECK(readBytes(regs::GYRO_XOUT_H, 6, buffer))) return err;
    *x = buffer[0] << 8 | buffer[1];
    *y = buffer[2] << 8 | buffer[3];
    *z = buffer[4] << 8 | buffer[5];
    return err;
}

/**
 * Read temperature raw data.
 * */
esp_err_t ICM::temperature(int16_t* temp)
{
    if (ICM_ERR_CHECK(readBytes(regs::TEMP_OUT_H, 2, buffer))) return err;
    *temp = buffer[0] << 8 | buffer[1];
    return err;
}

/**
 * @brief Read accelerometer and gyroscope data at once.
 * */
esp_err_t ICM::motion(raw_axes_t* accel, raw_axes_t* gyro)
{
    if (ICM_ERR_CHECK(readBytes(regs::ACCEL_XOUT_H, 14, buffer))) return err;
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];
    gyro->x  = buffer[8] << 8 | buffer[9];
    gyro->y  = buffer[10] << 8 | buffer[11];
    gyro->z  = buffer[12] << 8 | buffer[13];
    return err;
}

/**
 * @brief Configure the Interrupt pin (INT).
 * @param config configuration desired.
 */
esp_err_t ICM::setInterruptConfig(int_config_t config)
{
    if (ICM_ERR_CHECK(readByte(regs::INT_PIN_CONFIG, buffer))) return err;
    // zero the bits we're setting, but keep the others we're not setting as they are;
    constexpr uint8_t INT_PIN_CONFIG_BITMASK = (1 << regs::INT_CFG_LEVEL_BIT) | (1 << regs::INT_CFG_OPEN_BIT) |
                                               (1 << regs::INT_CFG_LATCH_EN_BIT) |
                                               (1 << regs::INT_CFG_ANYRD_2CLEAR_BIT);
    buffer[0] &= ~INT_PIN_CONFIG_BITMASK;
    // set the configurations
    buffer[0] |= config.level << regs::INT_CFG_LEVEL_BIT;
    buffer[0] |= config.drive << regs::INT_CFG_OPEN_BIT;
    buffer[0] |= config.mode << regs::INT_CFG_LATCH_EN_BIT;
    buffer[0] |= config.clear << regs::INT_CFG_ANYRD_2CLEAR_BIT;
    return ICM_ERR_CHECK(writeByte(regs::INT_PIN_CONFIG, buffer[0]));
}

/**
 * @brief Return Interrupt pin (INT) configuration.
 */
int_config_t ICM::getInterruptConfig()
{
    ICM_ERR_CHECK(readByte(regs::INT_PIN_CONFIG, buffer));
    int_config_t config{};
    config.level = (int_lvl_t)((buffer[0] >> regs::INT_CFG_LEVEL_BIT) & 0x1);
    config.drive = (int_drive_t)((buffer[0] >> regs::INT_CFG_OPEN_BIT) & 0x1);
    config.mode  = (int_mode_t)((buffer[0] >> regs::INT_CFG_LATCH_EN_BIT) & 0x1);
    config.clear = (int_clear_t)((buffer[0] >> regs::INT_CFG_ANYRD_2CLEAR_BIT) & 0x1);
    return config;
}

/**
 * @brief Enable features to generate signal at Interrupt pin
 * @param mask ORed features.
 */
esp_err_t ICM::setInterruptEnabled(int_en_t mask)
{
    return ICM_ERR_CHECK(writeByte(regs::INT_ENABLE, mask));
}

/**
 * @brief Return enabled features configured to generate signal at Interrupt pin.
 */
int_en_t ICM::getInterruptEnabled()
{
    ICM_ERR_CHECK(readByte(regs::INT_ENABLE, buffer));
    return (int_en_t) buffer[0];
}

/**
 * @brief Return the Interrupt status from INT_STATUS register.
 *
 * Note: Reading this register, clear all bits.
 */
int_stat_t ICM::getInterruptStatus()
{
    ICM_ERR_CHECK(readByte(regs::INT_STATUS, buffer));
    return (int_stat_t) buffer[0];
}

/**
 * @brief Change FIFO mode.
 *
 * Options:
 * `FIFO_MODE_OVERWRITE`: When the fifo is full, additional writes will be
 *  written to the fifo,replacing the oldest data.
 * `FIFO_MODE_STOP_FULL`: When the fifo is full, additional writes will not be written to fifo.
 * */
esp_err_t ICM::setFIFOMode(fifo_mode_t mode)
{
    return ICM_ERR_CHECK(writeBit(regs::CONFIG, regs::CONFIG_FIFO_MODE_BIT, mode));
}

/**
 * @brief Return FIFO mode.
 */
fifo_mode_t ICM::getFIFOMode()
{
    ICM_ERR_CHECK(readBit(regs::CONFIG, regs::CONFIG_FIFO_MODE_BIT, buffer));
    return (fifo_mode_t) buffer[0];
}

/**
 * @brief Configure the sensors that will be written to the FIFO.
 * */
esp_err_t ICM::setFIFOConfig(fifo_config_t config)
{
    if (ICM_ERR_CHECK(writeByte(regs::FIFO_EN, (uint8_t) config))) return err;
    return ICM_ERR_CHECK(writeBit(regs::I2C_MST_CTRL, regs::I2CMST_CTRL_SLV_3_FIFO_EN_BIT, config >> 8));
}

/**
 * @brief Return FIFO configuration.
 */
fifo_config_t ICM::getFIFOConfig()
{
    ICM_ERR_CHECK(readBytes(regs::FIFO_EN, 2, buffer));
    fifo_config_t config = buffer[0];
    config |= (buffer[1] & (1 << icm20601::regs::I2CMST_CTRL_SLV_3_FIFO_EN_BIT)) << 3;
    return config;
}

/**
 * @brief Enabled / disable FIFO module.
 * */
esp_err_t ICM::setFIFOEnabled(bool enable)
{
    return ICM_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_FIFO_EN_BIT, (uint8_t) enable));
}

/**
 * @brief Return FIFO module state.
 */
bool ICM::getFIFOEnabled()
{
    ICM_ERR_CHECK(readBit(regs::USER_CTRL, regs::USERCTRL_FIFO_EN_BIT, buffer));
    return buffer[0];
}

/**
 * @brief Reset FIFO module.
 *
 * Zero FIFO count, reset is asynchronous. \n
 * The bit auto clears after one clock cycle.
 * */
esp_err_t ICM::resetFIFO()
{
    return ICM_ERR_CHECK(writeBit(regs::USER_CTRL, regs::USERCTRL_FIFO_RESET_BIT, 1));
}

/**
 * @brief Return number of written bytes in the FIFO.
 * @note FIFO overflow generates an interrupt which can be check with getInterruptStatus().
 * */
uint16_t ICM::getFIFOCount()
{
    ICM_ERR_CHECK(readBytes(regs::FIFO_COUNT_H, 2, buffer));
    uint16_t count = buffer[0] << 8 | buffer[1];
    return count;
}

/**
 * @brief Read data contained in FIFO buffer.
 * */
esp_err_t ICM::readFIFO(size_t length, uint8_t* data)
{
    return ICM_ERR_CHECK(readBytes(regs::FIFO_R_W, length, data));
}

/**
 * @brief Write data to FIFO buffer.
 * */
esp_err_t ICM::writeFIFO(size_t length, const uint8_t* data)
{
    return ICM_ERR_CHECK(writeBytes(regs::FIFO_R_W, length, data));
}

/**
 * @brief Configure the active level of FSYNC pin that will cause an interrupt.
 * @details Use setFsyncEnabled() to enable / disable this interrupt.
 * */
esp_err_t ICM::setFsyncConfig(int_lvl_t level)
{
    return ICM_ERR_CHECK(writeBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_LEVEL_BIT, level));
}

/**
 * @brief Return FSYNC pin active level configuration.
 */
int_lvl_t ICM::getFsyncConfig()
{
    ICM_ERR_CHECK(readBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_LEVEL_BIT, buffer));
    return (int_lvl_t) buffer[0];
}

/**
 * @brief Enable / disable FSYNC pin to cause an interrupt.
 * @note
 * - The interrupt status is located in I2C_MST_STATUS register, so use
 *   the method getAuxI2CStatus() which reads this register to get FSYNC status.
 *   Keep in mind that a read from I2C_MST_STATUS register clears all its status bits,
 *   so take care to miss status bits when using Auxiliary I2C bus too.
 *
 * - It is possible to enable the FSYNC interrupt propagate to INT pin
 *   with setInterruptEnabled(), then the status can also be read with getInterruptStatus().
 *
 * @see setFsyncConfig().
 * */
esp_err_t ICM::setFsyncEnabled(bool enable)
{
    return ICM_ERR_CHECK(writeBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_INT_MODE_EN_BIT, enable));
}

/**
 * @brief Return FSYNC enable state.
 */
bool ICM::getFsyncEnabled()
{
    ICM_ERR_CHECK(readBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_INT_MODE_EN_BIT, buffer));
    return buffer[0];
}

/**
 * @brief Print out register values for debugging purposes.
 * @param start first register number.
 * @param end last register number.
 */
esp_err_t ICM::registerDump(uint8_t start, uint8_t end)
{
    constexpr uint8_t kNumOfRegs = 128;
    if (end - start < 0 || start >= kNumOfRegs || end >= kNumOfRegs) return err = ESP_FAIL;
    // printf(LOG_COLOR_W ">> " CONFIG_MPU_CHIP_MODEL " register dump:" LOG_RESET_COLOR "\n");
    uint8_t data;
    for (int i = start; i <= end; i++) {
        if (ICM_ERR_CHECK(readByte(i, &data))) {
            ICM_LOGEMSG("", "Reading Error.");
            return err;
        }
        printf("MPU: reg[ 0x%s%X ]  data( 0x%s%X )\n", i < 0x10 ? "0" : "", i, data < 0x10 ? "0" : "", data);
    }
    return err;
}

/**
 * @brief Trigger gyro and accel hardware self-test.
 * @attention when calling this function, the MPU must remain as horizontal as possible (0 degrees), facing up.
 * @param result Should be ZERO if gyro and accel passed.
 * @todo Elaborate doc.
 * */
esp_err_t ICM::selfTest(selftest_t* result)
{
    constexpr accel_fs_t kAccelFS = ACCEL_FS_4G;
    constexpr gyro_fs_t kGyroFS   = GYRO_FS_500DPS;
    raw_axes_t gyroRegBias, accelRegBias;
    raw_axes_t gyroSTBias, accelSTBias;
    // get regular biases
    if (ICM_ERR_CHECK(getBiases(kAccelFS, kGyroFS, &accelRegBias, &gyroRegBias, false))) return err;
    // get self-test biases
    if (ICM_ERR_CHECK(getBiases(kAccelFS, kGyroFS, &accelSTBias, &gyroSTBias, true))) return err;
    // perform self-tests
    uint8_t accelST, gyroST;
    if (ICM_ERR_CHECK(accelSelfTest(accelRegBias, accelSTBias, &accelST))) return err;
    if (ICM_ERR_CHECK(gyroSelfTest(gyroRegBias, gyroSTBias, &gyroST))) return err;
    // check results
    *result = 0;
    if (accelST != 0) *result |= SELF_TEST_ACCEL_FAIL;
    if (gyroST != 0) *result |= SELF_TEST_GYRO_FAIL;
    return err;
}

// Production Self-Test table for MPU6500 based models,
// used in accel and gyro self-test code below.
static constexpr uint16_t kSelfTestTable[256] = {
    2620,  2646,  2672,  2699,  2726,  2753,  2781,  2808,   // 7
    2837,  2865,  2894,  2923,  2952,  2981,  3011,  3041,   // 15
    3072,  3102,  3133,  3165,  3196,  3228,  3261,  3293,   // 23
    3326,  3359,  3393,  3427,  3461,  3496,  3531,  3566,   // 31
    3602,  3638,  3674,  3711,  3748,  3786,  3823,  3862,   // 39
    3900,  3939,  3979,  4019,  4059,  4099,  4140,  4182,   // 47
    4224,  4266,  4308,  4352,  4395,  4439,  4483,  4528,   // 55
    4574,  4619,  4665,  4712,  4759,  4807,  4855,  4903,   // 63
    4953,  5002,  5052,  5103,  5154,  5205,  5257,  5310,   // 71
    5363,  5417,  5471,  5525,  5581,  5636,  5693,  5750,   // 79
    5807,  5865,  5924,  5983,  6043,  6104,  6165,  6226,   // 87
    6289,  6351,  6415,  6479,  6544,  6609,  6675,  6742,   // 95
    6810,  6878,  6946,  7016,  7086,  7157,  7229,  7301,   // 103
    7374,  7448,  7522,  7597,  7673,  7750,  7828,  7906,   // 111
    7985,  8065,  8145,  8227,  8309,  8392,  8476,  8561,   // 119
    8647,  8733,  8820,  8909,  8998,  9088,  9178,  9270,   //
    9363,  9457,  9551,  9647,  9743,  9841,  9939,  10038,  //
    10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,  //
    10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,  //
    11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,  //
    12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,  //
    13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,  //
    15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,  //
    16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,  //
    17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,  //
    19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,  //
    20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,  //
    22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,  //
    24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,  //
    26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,  //
    28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,  //
    30903, 31212, 31524, 31839, 32157, 32479, 32804, 33132   //
};

/**
 * @brief Accel Self-test.
 * @param result self-test error for each axis (X=bit0, Y=bit1, Z=bit2). Zero is a pass.
 * @note Bias should be in 16G format for MPU6050 and 2G for MPU6500 based models.
 * */
esp_err_t ICM::accelSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result)
{
    constexpr accel_fs_t kAccelFS = ACCEL_FS_4G;
    // Criteria A: must be within 50% variation
    constexpr float kMaxVariation = .5f;
    // Criteria B: must be between 255 mg and 675 mg
    constexpr float kMinGravity = .225f, kMaxGravity = .675f;
    // Criteria C: 500 mg for accel
    constexpr float kMaxGravityOffset = .5f;

    /* Convert biases */
    float_axes_t regularBiasGravity  = math::accelGravity(regularBias, kAccelFS);
    float_axes_t selfTestBiasGravity = math::accelGravity(selfTestBias, kAccelFS);
    ICM_LOGVMSG(msgs::EMPTY, "regularBias: %+d %+d %+d | regularBiasGravity: %+.2f %+.2f %+.2f", regularBias.x,
                regularBias.y, regularBias.z, regularBiasGravity.x, regularBiasGravity.y, regularBiasGravity.z);
    ICM_LOGVMSG(msgs::EMPTY, "selfTestBias: %+d %+d %+d | selfTestBiasGravity: %+.2f %+.2f %+.2f", selfTestBias.x,
                selfTestBias.y, selfTestBias.z, selfTestBiasGravity.x, selfTestBiasGravity.y, selfTestBiasGravity.z);

    /* Get OTP production shift code */
    uint8_t shiftCode[3];
    if (ICM_ERR_CHECK(readBytes(regs::SELF_TEST_X_ACCEL, 3, shiftCode))) return err;
    ICM_LOGVMSG(msgs::EMPTY, "shiftCode: %+d %+d %+d", shiftCode[0], shiftCode[1], shiftCode[2]);

    /* Calulate production shift value */
    float shiftProduction[3] = {0};
    for (int i = 0; i < 3; i++) {
        if (shiftCode[i] != 0) {
            shiftProduction[i] = kSelfTestTable[shiftCode[i] - 1];
            shiftProduction[i] /= math::accelSensitivity(ACCEL_FS_4G);
        }
    }
    ICM_LOGVMSG(msgs::EMPTY, "shiftProduction: %+.2f %+.2f %+.2f", shiftProduction[0], shiftProduction[1],
                shiftProduction[2]);

    /* Evaluate criterias */
    *result                 = 0;
    float shiftResponse[3]  = {0};
    float shiftVariation[3] = {0};
    for (int i = 0; i < 3; i++) {
        shiftResponse[i] = fabs(selfTestBiasGravity[i] - regularBiasGravity[i]);
        // Criteria A
        if (shiftProduction[i] != 0) {
            shiftVariation[i] = shiftResponse[i] / shiftProduction[i] - 1;
            if (fabs(shiftVariation[i]) > kMaxVariation) *result |= 1 << i;
            // Criteria B
        }
        else if (shiftResponse[i] < kMinGravity || shiftResponse[i] > kMaxGravity) {
            *result |= 1 << i;
        }
// Criteria C
        if (fabs(regularBiasGravity[i] > kMaxGravityOffset)) *result |= 1 << i;
    }
    ICM_LOGVMSG(msgs::EMPTY, "shiftResponse: %+.2f %+.2f %+.2f", shiftResponse[0], shiftResponse[1], shiftResponse[2]);
    ICM_LOGVMSG(msgs::EMPTY, "shiftVariation: %+.2f %+.2f %+.2f", shiftVariation[0], shiftVariation[1],
                shiftVariation[2]);

    ICM_LOGD("Accel self-test: [X=%s] [Y=%s] [Z=%s]", ((*result & 0x1) ? "FAIL" : "OK"),
             ((*result & 0x2) ? "FAIL" : "OK"), ((*result & 0x4) ? "FAIL" : "OK"));
    return err;
}

/**
 * @brief Gyro Self-test.
 * @param result Self-test error for each axis (X=bit0, Y=bit1, Z=bit2). Zero is a pass.
 * @note Bias should be in 250DPS format for both MPU6050 and MPU6500 based models.
 * */
esp_err_t ICM::gyroSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result)
{
    constexpr gyro_fs_t kGyroFS = GYRO_FS_500DPS;

    // Criteria A: must be within 50% variation
    constexpr float kMaxVariation = .5f;
    // Criteria B: must be between 20 dps and 60 dps
    constexpr float kMinDPS = 20.f, kMaxDPS = 60.f;

    /* Convert biases */
    float_axes_t regularBiasDPS  = math::gyroDegPerSec(regularBias, kGyroFS);
    float_axes_t selfTestBiasDPS = math::gyroDegPerSec(selfTestBias, kGyroFS);
    ICM_LOGVMSG(msgs::EMPTY, "regularBias: %+d %+d %+d | regularBiasDPS: %+.2f %+.2f %+.2f", regularBias.x,
                regularBias.y, regularBias.z, regularBiasDPS.x, regularBiasDPS.y, regularBiasDPS.z);
    ICM_LOGVMSG(msgs::EMPTY, "selfTestBias: %+d %+d %+d | selfTestBiasDPS: %+.2f %+.2f %+.2f", selfTestBias.x,
                selfTestBias.y, selfTestBias.z, selfTestBiasDPS.x, selfTestBiasDPS.y, selfTestBiasDPS.z);

    /* Get OTP production shift code */
    uint8_t shiftCode[3];
    if (ICM_ERR_CHECK(readBytes(regs::SELF_TEST_X_GYRO, 3, shiftCode))) return err;
    ICM_LOGVMSG(msgs::EMPTY, "shiftCode: %+d %+d %+d", shiftCode[0], shiftCode[1], shiftCode[2]);

    /* Calulate production shift value */
    float shiftProduction[3] = {0};
    for (int i = 0; i < 3; i++) {
        if (shiftCode[i] != 0) {
            shiftProduction[i] = kSelfTestTable[shiftCode[i] - 1];
            shiftProduction[i] /= math::gyroSensitivity(kGyroFS);
        }
    }
    ICM_LOGVMSG(msgs::EMPTY, "shiftProduction: %+.2f %+.2f %+.2f", shiftProduction[0], shiftProduction[1],
                shiftProduction[2]);

    /* Evaluate criterias */
    *result                 = 0;
    float shiftResponse[3]  = {0};
    float shiftVariation[3] = {0};
    for (int i = 0; i < 3; i++) {
        shiftResponse[i] = fabs(selfTestBiasDPS[i] - regularBiasDPS[i]);
        // Criteria A
        if (shiftProduction[i] != 0) {
            shiftVariation[i] = shiftResponse[i] / shiftProduction[i] - 1;
            if (fabs(shiftVariation[i]) > kMaxVariation) *result |= 1 << i;
            // Criteria B
        }
        else if (shiftResponse[i] < kMinDPS || shiftResponse[i] > kMaxDPS) {
            *result |= 1 << i;
        }
    }
    ICM_LOGVMSG(msgs::EMPTY, "shiftResponse: %+.2f %+.2f %+.2f", shiftResponse[0], shiftResponse[1], shiftResponse[2]);
    ICM_LOGVMSG(msgs::EMPTY, "shiftVariation: %+.2f %+.2f %+.2f", shiftVariation[0], shiftVariation[1],
                shiftVariation[2]);

    ICM_LOGD("Gyro self-test: [X=%s] [Y=%s] [Z=%s]", ((*result & 0x1) ? "FAIL" : "OK"),
             ((*result & 0x2) ? "FAIL" : "OK"), ((*result & 0x4) ? "FAIL" : "OK"));
    return err;
}

/**
 * @brief Compute the Biases in regular mode and self-test mode.
 * @attention When calculating the biases the MPU must remain as horizontal as possible (0 degrees), facing up.
 * This algorithm takes about ~400ms to compute offsets.
 * */
esp_err_t ICM::getBiases(accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                         bool selftest)
{
    // configurations to compute biases
    constexpr uint16_t kSampleRate      = 1000;
    constexpr dlpf_t kDLPF              = DLPF_188HZ;
    constexpr fifo_config_t kFIFOConfig = FIFO_CFG_ACCEL | FIFO_CFG_GYRO;
    constexpr size_t kPacketSize        = 12;
    // backup previous configuration
    const uint16_t prevSampleRate      = getSampleRate();
    const dlpf_t prevDLPF              = getDigitalLowPassFilter();
    const accel_fs_t prevAccelFS       = getAccelFullScale();
    const gyro_fs_t prevGyroFS         = getGyroFullScale();
    const fifo_config_t prevFIFOConfig = getFIFOConfig();
    const bool prevFIFOState           = getFIFOEnabled();
    // setup
    if (ICM_ERR_CHECK(setSampleRate(kSampleRate))) return err;
    if (ICM_ERR_CHECK(setDigitalLowPassFilter(kDLPF))) return err;
    if (ICM_ERR_CHECK(setAccelFullScale(accelFS))) return err;
    if (ICM_ERR_CHECK(setGyroFullScale(gyroFS))) return err;
    if (ICM_ERR_CHECK(setFIFOConfig(kFIFOConfig))) return err;
    if (ICM_ERR_CHECK(setFIFOEnabled(true))) return err;
    if (selftest) {
        if (ICM_ERR_CHECK(writeBits(regs::ACCEL_CONFIG, regs::ACONFIG_XA_ST_BIT, 3, 0x7))) {
            return err;
        }
        if (ICM_ERR_CHECK(writeBits(regs::GYRO_CONFIG, regs::GCONFIG_XG_ST_BIT, 3, 0x7))) {
            return err;
        }
    }
    // wait for 200ms for sensors to stabilize
    vTaskDelay(200 / portTICK_PERIOD_MS);
    // fill FIFO for 100ms
    if (ICM_ERR_CHECK(resetFIFO())) return err;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (ICM_ERR_CHECK(setFIFOConfig(FIFO_CFG_NONE))) return err;
    // get FIFO count
    const uint16_t fifoCount = getFIFOCount();
    if (ICM_ERR_CHECK(lastError())) return err;
    const int packetCount = fifoCount / kPacketSize;
    // read overrun bytes, if any
    const int overrunCount      = fifoCount - (packetCount * kPacketSize);
    uint8_t buffer[kPacketSize] = {0};
    if (overrunCount > 0) {
        if (ICM_ERR_CHECK(readFIFO(overrunCount, buffer))) return err;
    }
    // fetch data and add up
    axes_t<int> accelAvg, gyroAvg;
    for (int i = 0; i < packetCount; i++) {
        if (ICM_ERR_CHECK(readFIFO(kPacketSize, buffer))) return err;
        // retrieve data
        raw_axes_t accelCur, gyroCur;
        accelCur.x = (buffer[0] << 8) | buffer[1];
        accelCur.y = (buffer[2] << 8) | buffer[3];
        accelCur.z = (buffer[4] << 8) | buffer[5];
        gyroCur.x  = (buffer[6] << 8) | buffer[7];
        gyroCur.y  = (buffer[8] << 8) | buffer[9];
        gyroCur.z  = (buffer[10] << 8) | buffer[11];
        // add up
        accelAvg.x += accelCur.x;
        accelAvg.y += accelCur.y;
        accelAvg.z += accelCur.z;
        gyroAvg.x += gyroCur.x;
        gyroAvg.y += gyroCur.y;
        gyroAvg.z += gyroCur.z;
    }
    // calculate average
    accelAvg.x /= packetCount;
    accelAvg.y /= packetCount;
    accelAvg.z /= packetCount;
    gyroAvg.x /= packetCount;
    gyroAvg.y /= packetCount;
    gyroAvg.z /= packetCount;
    // remove gravity from Accel Z axis
    const uint16_t gravityLSB = INT16_MAX >> (accelFS + 1);
    accelAvg.z -= gravityLSB;
    // save biases
    for (int i = 0; i < 3; i++) {
        (*accelBias)[i] = (int16_t) accelAvg[i];
        (*gyroBias)[i]  = (int16_t) gyroAvg[i];
    }
    // set back previous configs
    if (ICM_ERR_CHECK(setSampleRate(prevSampleRate))) return err;
    if (ICM_ERR_CHECK(setDigitalLowPassFilter(prevDLPF))) return err;
    if (ICM_ERR_CHECK(setAccelFullScale(prevAccelFS))) return err;
    if (ICM_ERR_CHECK(setGyroFullScale(prevGyroFS))) return err;
    if (ICM_ERR_CHECK(setFIFOConfig(prevFIFOConfig))) return err;
    if (ICM_ERR_CHECK(setFIFOEnabled(prevFIFOState))) return err;
    return err;
}

}  // namespace icm20601