// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================


#ifndef _ICM_LOG_HPP_
#define _ICM_LOG_HPP_

#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Note: declare TAG before include this header
// Note: include only in .cpp files from this library

#define ICM_LOGE(format, ...) { ESP_LOGE(TAG, format, ##__VA_ARGS__); }
#define ICM_LOGW(format, ...) { ESP_LOGW(TAG, format, ##__VA_ARGS__); }
#define ICM_LOGI(format, ...) { ESP_LOGI(TAG, format, ##__VA_ARGS__); }
#define ICM_LOGD(format, ...) { ESP_LOGD(TAG, format, ##__VA_ARGS__); }
#define ICM_LOGV(format, ...) { ESP_LOGV(TAG, format, ##__VA_ARGS__); }

#define ICM_LOGEMSG(msg, format, ...) ICM_LOGE("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)
#define ICM_LOGWMSG(msg, format, ...) ICM_LOGW("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)
#define ICM_LOGIMSG(msg, format, ...) ICM_LOGI("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)
#define ICM_LOGDMSG(msg, format, ...) ICM_LOGD("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)
#define ICM_LOGVMSG(msg, format, ...) ICM_LOGV("%s()-> %s" format, __FUNCTION__, msg, ##__VA_ARGS__)


#define ICM_ERR_CHECK(x) (x)

/*! ICM 20601 Driver namespace */
namespace icm20601
{
/*! Log namespace */
inline namespace log
{
/*! Messages namespace */
namespace msgs
{
static const char INVALID_ARG[]           = "Invalid Argument";
static const char INVALID_STATE[]         = "Invalid State";
static const char INVALID_LENGTH[]        = "Invalid length";
static const char INVALID_FIFO_RATE[]     = "Invalid FIFO rate";
static const char INVALID_SAMPLE_RATE[]   = "Invalid Sample rate";
static const char INVALID_TAP_THRESH[]    = "Invalid Tap threshold";
static const char DMP_LOAD_FAIL[]         = "Failed to load DMP firmware";
static const char DMP_NOT_LOADED[]        = "DMP firmware has not been loaded";
static const char UNKNOWN_DMP_CFG_STATE[] = "Unknown DMP config state";
static const char NO_AXIS_PASSED[]        = "No Axis passed";
static const char BANK_BOUNDARIES[]       = "Bank boundaries overpass";
static const char FIFO_CORRUPTION[]       = "FIFO Corruption. Quaternion data outside of the acceptable threshold";
static const char AUX_I2C_DISABLED[]      = "Auxiliary I2C is disabled";
static const char AUX_I2C_SLAVE_NACK[]    = "Auxiliary I2C Slave NACK";
static const char AUX_I2C_LOST_ARB[]      = "Auxiliary I2C Master loose abitraion of the bus";
static const char COMPASS_DISABLED[]      = "Compass is disabled";
static const char NOT_SUPPORTED[]         = "Not supported";
static const char TIMEOUT[]               = "Timeout";
static const char EMPTY[]                 = "";

}  // namespace msgs

static inline esp_err_t errorCheckLogger(esp_err_t x, const char* func, const int line, const char* expr)
{
    if (x) ICM_LOGE("func:%s @ line:%d, expr:\"%s\", error:0x%X ", func, line, expr, x);
    return x;
}

}  // namespace log

}  // namespace icm20601

#endif /* end of include guard: _ICM_LFOG_HPP_ */