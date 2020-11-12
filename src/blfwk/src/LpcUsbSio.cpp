/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "blfwk/Logging.h"
#include "blfwk/LpcUsbSio.h"
#include "blfwk/format_string.h"

using namespace blfwk;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! This macro converts a value from KHz to Hz.
#define CONVERT_KHZ_TO_HZ(KHz) (KHz * 1000)

//! This struct maps an error code to a description of that code.
struct LpcUsbSioErrorMessage
{
    LPCUSBSIO_ERR_T code;      //!< Error code value.
    const std::string message; //!< Description of the status.
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! This variable containing error codes and corresponding descriptions of the codes.
LpcUsbSioErrorMessage s_errorMessage[] = {
    { LPCUSBSIO_OK, "Success" },
    { LPCUSBSIO_ERR_HID_LIB, "HID library error" },
    { LPCUSBSIO_ERR_BAD_HANDLE, "Handle passed to the function is invalid" },
    { LPCUSBSIO_ERR_SYNCHRONIZATION, "Thread Synchronization error" },
    { LPCUSBSIO_ERR_MEM_ALLOC, "Memory Allocation error" },
    { LPCUSBSIO_ERR_MUTEX_CREATE, "Mutex Creation error" },
    { LPCUSBSIO_ERR_FATAL, "Fatal error occurred" },
    { LPCUSBSIO_ERR_I2C_NAK, "Transfer aborted due to NACK" },
    { LPCUSBSIO_ERR_I2C_BUS, "Transfer aborted due to bus error" },
    { LPCUSBSIO_ERR_I2C_SLAVE_NAK, "NAK received after SLA+W or SLA+R" },
    { LPCUSBSIO_ERR_I2C_ARBLOST, "I2C bus arbitration lost to other master" },
    { LPCUSBSIO_ERR_TIMEOUT, "Transaction timed out" },
    { LPCUSBSIO_ERR_INVALID_CMD, "Invalid HID_SIO Request or Request not supported in this version" },
    { LPCUSBSIO_ERR_INVALID_PARAM, "Invalid parameters are provided for the given Request" },
    { LPCUSBSIO_ERR_PARTIAL_DATA, "Partial transfer completed" },
};

////////////////////////////////////////////////////////////////////////////////
// Code
///////////////////////////////////////////////////////////////////////////////

// See LpcUsbSio.h for documentation of this method.
LpcUsbSio::LpcUsbSio(const LpcUsbSio::LpcUsbSioPortConfigData &config)
    : m_usbVid(config.usbVid)
    , m_usbPid(config.usbPid)
    , m_usbString(config.usbString)
    , m_usbPath(config.usbPath)
    , m_portIndex(config.portIndex)
{
    m_portCount = LPCUSBSIO_GetNumPorts(m_usbVid, m_usbPid);
    if (m_portCount < 1)
    {
        throw std::runtime_error(format_string("Error: No LPC USB Serial I/O is detected.\n"));
    }
    else if (m_portCount < (m_portIndex + 1))
    {
        throw std::runtime_error(
            format_string("Error: Invalid index %d, only %d port(s) available.\n", m_portIndex, m_portCount));
    }

    m_portHandler = LPCUSBSIO_Open(m_portIndex);
    if (m_portHandler == NULL)
    {
        throw std::runtime_error(
            format_string("Error: Failed to open the LPC USB Serial I/O port index %x.\n", m_portIndex));
    }
}

// See LpcUsbSio.h for documentation of this method.
LpcUsbSio::~LpcUsbSio()
{
    if (m_portHandler)
    {
        LPCUSBSIO_Close(m_portHandler);
        m_portHandler = NULL;
    }
}

// See LpcUsbSio.h for documentation of this method.
bool LpcUsbSio::parse(const string_vector_t &params, LpcUsbSio::LpcUsbSioConfigData &config)
{
    if (!params[0].compare(0, 3, "spi"))
    {
        config.function = LpcUsbSio::kFunction_SPI;

        // Currently only support one SPI port. Can be extended in the furture.
        config.spiConfig.spiPortIndex = 0;

        if (params.size() > 1)
        {
            int32_t port = atoi(params[1].c_str());
            if ((port < 0) || (port > 7))
                return false;

            config.spiConfig.spiSselPort = (uint8_t)port;
        }
        if (params.size() > 2)
        {
            int32_t pin = atoi(params[2].c_str());
            if ((pin < 0) || (pin > 0x1F))
                return false;

            config.spiConfig.spiSselPin = (uint8_t)pin;
        }
        if (params.size() > 3)
        {
            int32_t spiSpeed = CONVERT_KHZ_TO_HZ(atoi(params[3].c_str()));
            if (spiSpeed <= 0)
                return false;

            config.spiConfig.spiSpeedHz = spiSpeed;
        }
        if (params.size() > 4)
        {
            config.spiConfig.spiPolarity = (LpcUsbSio::spi_clock_polarity_t)atoi(params[4].c_str());
        }
        if (params.size() > 5)
        {
            config.spiConfig.spiPhase = (LpcUsbSio::spi_clock_phase_t)atoi(params[5].c_str());
        }
    }
    else if (!params[0].compare(0, 3, "i2c"))
    {
        config.function = LpcUsbSio::kFunction_I2C;

        // Currently only support one I2C port. Can be extended in the furture.
        config.i2cConfig.i2cPortIndex = 0;

        if (params.size() > 1)
        {
            uint32_t i2cAddress = strtoul(params[1].c_str(), NULL, 16);

            if (i2cAddress > 0x7F)
            {
                i2cAddress &= 0x7F;
                Log::warning("Warning: Only 7-bit i2c address is supported, so the effective value is 0x%x\n",
                             i2cAddress);
            }
            config.i2cConfig.i2cAddress = (uint8_t)i2cAddress;
        }
        if (params.size() > 2)
        {
            int32_t i2cSpeed = CONVERT_KHZ_TO_HZ(atoi(params[2].c_str()));
            if (i2cSpeed <= 0)
            {
                return false;
            }

            if (i2cSpeed < I2C_CLOCK_FAST_MODE)
            {
                config.i2cConfig.i2cSpeedHz = I2C_CLOCK_STANDARD_MODE;
            }
            else if (i2cSpeed < I2C_CLOCK_FAST_MODE_PLUS)
            {
                config.i2cConfig.i2cSpeedHz = I2C_CLOCK_FAST_MODE;
            }
            else
            {
                config.i2cConfig.i2cSpeedHz = I2C_CLOCK_FAST_MODE_PLUS;
            }

            if ((i2cSpeed != I2C_CLOCK_STANDARD_MODE) && (i2cSpeed != I2C_CLOCK_FAST_MODE) &&
                (i2cSpeed != I2C_CLOCK_FAST_MODE_PLUS))
            {
                Log::warning(
                    "Warning: Only 100Kbps, 400Kbps and 1Mbps are supported, so the effective speed is %dbps\n",
                    config.i2cConfig.i2cSpeedHz);
            }
        }
    }
    else if (!params[0].compare(0, 4, "gpio"))
    {
        // Not supported yet.
        // config.function = LpcUsbSio::kFunction_GPIO;
        return false;
    }
    else
    {
        // Error: Invalid BusPal function.
        return false;
    }

    return true;
}

// See LpcUsbSio.h for documentation of this method.
const std::string LpcUsbSio::getError(LPCUSBSIO_ERR_T errorCode)
{
    int i;
    for (i = 0; i < (sizeof(s_errorMessage) / sizeof(s_errorMessage[0])); ++i)
    {
        if (errorCode == s_errorMessage[i].code)
        {
            return s_errorMessage[i].message;
        }
    }

    return format_string("Unknown error code (%d)", errorCode);
}

// See LpcUsbSio.h for documentation of this method.
LpcUsbSioSpi::LpcUsbSioSpi(const LpcUsbSio::LpcUsbSioPortConfigData &portConfig,
                           const LpcUsbSio::LpcUsbSioSpiConfigData &spiConfig)
    : LpcUsbSio(portConfig)
    , m_spiPortIndex(spiConfig.spiPortIndex)
    , m_spiSselPort(spiConfig.spiSselPort)
    , m_spiSselPin(spiConfig.spiSselPin)
{
    m_spiPortCount = (int32_t)LPCUSBSIO_GetNumSPIPorts(m_portHandler);
    if (m_spiPortCount < 0)
    {
        throw std::runtime_error(
            format_string("Error: Failed to get SPI port count, due to %s.\n", LPCUSBSIO_Error(m_portHandler)));
    }
    else if (m_spiPortCount == 0)
    {
        throw std::runtime_error(format_string("Error: No SPI port is detected on current LPC USB Serial I/O.\n"));
    }
    else if (m_spiPortCount < (m_spiPortIndex + 1))
    {
        throw std::runtime_error(
            format_string("Error: Invalid index %d, only %d port(s) available.\n", m_spiPortIndex, m_spiPortCount));
    }

    HID_SPI_PORTCONFIG_T spiParam;
    spiParam.busSpeed = spiConfig.spiSpeedHz;
    spiParam.Options = HID_SPI_CONFIG_OPTION_DATA_SIZE_8 | HID_SPI_CONFIG_OPTION_POL_0 | HID_SPI_CONFIG_OPTION_PHA_0 |
                       HID_SPI_CONFIG_OPTION_PRE_DELAY(0) | HID_SPI_CONFIG_OPTION_POST_DELAY(0);
    if (spiConfig.spiPolarity == LpcUsbSio::kSpiPolarity_ActiveLow)
    {
        spiParam.Options |= HID_SPI_CONFIG_OPTION_POL_1;
    }
    if (spiConfig.spiPhase == LpcUsbSio::kSpiPhase_SecondEdge)
    {
        spiParam.Options |= HID_SPI_CONFIG_OPTION_PHA_1;
    }

    m_spiPortHandler = SPI_Open(m_portHandler, &spiParam, m_spiPortIndex);
    if (m_spiPortHandler == NULL)
    {
        throw std::runtime_error(
            format_string("Error: Failed to open SPI port, due to %s.\n", LPCUSBSIO_Error(m_portHandler)));
    }
}

// See LpcUsbSio.h for documentation of this method.
LpcUsbSioSpi::~LpcUsbSioSpi()
{
    if (m_spiPortHandler)
    {
        SPI_Close(m_spiPortHandler);
        m_spiPortHandler = NULL;
    }
}

// See LpcUsbSio.h for documentation of this method.
uint32_t LpcUsbSioSpi::write(const uint8_t *data, uint32_t byteCount)
{
    uint32_t total = 0;
    uint8_t *rxBuffer = (uint8_t *)malloc(byteCount);

    while (byteCount)
    {
        uint16_t byteToTransfer = 0;
        if (byteCount > UINT16_MAX)
        {
            byteToTransfer = UINT16_MAX;
        }
        else
        {
            byteToTransfer = (uint16_t)byteCount;
        }

        SPI_XFER_T xfer;
        xfer.options = 0;
        xfer.length = byteToTransfer;
        xfer.txBuff = data + total;
        xfer.rxBuff = rxBuffer + total;
        xfer.device = LPCUSBSIO_GEN_SPI_DEVICE_NUM(m_spiSselPort, m_spiSselPin);

        int result = SPI_Transfer(m_spiPortHandler, &xfer);
        if (result < 0)
        {
            Log::error("Error: Failed to write data via SPI port, due to %x.\n",
                       LpcUsbSio::getError((LPCUSBSIO_ERR_T)result).c_str());
            break;
        }
        else
        {
            byteCount -= result;
            total += result;
        }
    }

    free(rxBuffer);

    return total;
}

// See LpcUsbSio.h for documentation of this method.
uint32_t LpcUsbSioSpi::read(uint8_t *data, uint32_t byteCount)
{
    uint32_t total = 0;
    uint8_t *txBuffer = (uint8_t *)malloc(byteCount);
    memset(txBuffer, 0x0, byteCount);

    while (byteCount)
    {
        uint16_t byteToTransfer = 0;
        if (byteCount > UINT16_MAX)
        {
            byteToTransfer = UINT16_MAX;
        }
        else
        {
            byteToTransfer = (uint16_t)byteCount;
        }

        SPI_XFER_T xfer;
        xfer.options = 0;
        xfer.length = byteToTransfer;
        xfer.txBuff = txBuffer + total;
        xfer.rxBuff = data + total;
        xfer.device = LPCUSBSIO_GEN_SPI_DEVICE_NUM(m_spiSselPort, m_spiSselPin);

        int result = SPI_Transfer(m_spiPortHandler, &xfer);
        if (result < 0)
        {
            Log::error("Error: Failed to read data via SPI port, due to %s.\n",
                       LpcUsbSio::getError((LPCUSBSIO_ERR_T)result).c_str());
            break;
        }
        else
        {
            byteCount -= result;
            total += result;
        }
    }

    free(txBuffer);

    return total;
}

// See LpcUsbSio.h for documentation of this method.
LpcUsbSioI2c::LpcUsbSioI2c(const LpcUsbSio::LpcUsbSioPortConfigData &portConfig,
                           const LpcUsbSio::LpcUsbSioI2cConfigData &i2cConfig)
    : LpcUsbSio(portConfig)
    , m_i2cPortIndex(i2cConfig.i2cPortIndex)
    , m_i2cAddress(i2cConfig.i2cAddress)
{
    m_i2cPortCount = (int32_t)LPCUSBSIO_GetNumSPIPorts(m_portHandler);
    if (m_i2cPortCount == 0)
    {
        throw std::runtime_error(format_string("Error: No I2C port is detected on current LPC USB Serial I/O.\n"));
    }
    else if (m_i2cPortCount < 0)
    {
        throw std::runtime_error(
            format_string("Error: Failed to get I2C port count, due to %s.\n", LPCUSBSIO_Error(m_portHandler)));
    }
    else if (m_i2cPortCount < (m_i2cPortIndex + 1))
    {
        throw std::runtime_error(
            format_string("Error: Invalid index %d, only %d port(s) available.\n", m_i2cPortIndex, m_i2cPortCount));
    }

    I2C_PORTCONFIG_T i2cParam;
    i2cParam.ClockRate = i2cConfig.i2cSpeedHz;
    i2cParam.Options = 0;
    m_i2cPortHandler = I2C_Open(m_portHandler, &i2cParam, m_i2cPortIndex);
    if (m_i2cPortHandler == NULL)
    {
        throw std::runtime_error(
            format_string("Error: Failed to open I2C port, due to %s.\n", LPCUSBSIO_Error(m_portHandler)));
    }
}

// See LpcUsbSio.h for documentation of this method.
LpcUsbSioI2c::~LpcUsbSioI2c()
{
    if (m_i2cPortHandler)
    {
        I2C_Close(m_i2cPortHandler);
        m_i2cPortHandler = NULL;
    }
}

// See LpcUsbSio.h for documentation of this method.
uint32_t LpcUsbSioI2c::write(const uint8_t *data, uint32_t byteCount)
{
    uint32_t total = 0;

    while (byteCount)
    {
        uint16_t byteToTransfer = 0;
        if (byteCount > UINT16_MAX)
        {
            byteToTransfer = UINT16_MAX;
        }
        else
        {
            byteToTransfer = (uint16_t)byteCount;
        }

        int result = I2C_DeviceWrite(m_i2cPortHandler, m_i2cAddress, (uint8_t *)data + total, byteToTransfer,
                                     I2C_TRANSFER_OPTIONS_START_BIT | I2C_TRANSFER_OPTIONS_STOP_BIT |
                                         I2C_TRANSFER_OPTIONS_BREAK_ON_NACK | I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE);
        if (result < 0)
        {
            Log::error("Error: Failed to write data via I2C port, due to %s.\n",
                       LpcUsbSio::getError((LPCUSBSIO_ERR_T)result).c_str());
            break;
        }
        else
        {
            byteCount -= result;
            total += result;
        }
    }

    return total;
}

// See LpcUsbSio.h for documentation of this method.
uint32_t LpcUsbSioI2c::read(uint8_t *data, uint32_t byteCount)
{
    uint32_t total = 0;

    while (byteCount)
    {
        uint16_t byteToTransfer = 0;
        if (byteCount > UINT16_MAX)
        {
            byteToTransfer = UINT16_MAX;
        }
        else
        {
            byteToTransfer = (uint16_t)byteCount;
        }

        int result = I2C_DeviceRead(m_i2cPortHandler, m_i2cAddress, (uint8_t *)data + total, byteToTransfer,
                                    I2C_TRANSFER_OPTIONS_START_BIT | I2C_TRANSFER_OPTIONS_STOP_BIT |
                                        I2C_TRANSFER_OPTIONS_BREAK_ON_NACK | I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE);
        if (result < 0)
        {
            Log::error("Error: Failed to read data via I2C port, due to %s.\n",
                       LpcUsbSio::getError((LPCUSBSIO_ERR_T)result).c_str());
            break;
        }
        else
        {
            byteCount -= result;
            total += result;
        }
    }

    return total;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
