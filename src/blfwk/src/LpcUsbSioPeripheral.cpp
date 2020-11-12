/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "blfwk/Logging.h"
#include "blfwk/LpcUsbSioPeripheral.h"
#include "blfwk/format_string.h"

using namespace blfwk;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See LpcUsbSioPeripheral.h for documentation of this method.
LpcUsbSioPeripheral::LpcUsbSioPeripheral(const LpcUsbSio::LpcUsbSioConfigData &config)
    : Peripheral()
{
    if (config.function == LpcUsbSio::kFunction_SPI)
    {
        m_lpcUsbSio = new LpcUsbSioSpi(config.portConfig, config.spiConfig);
    }
    else if (config.function == LpcUsbSio::kFunction_I2C)
    {
        m_lpcUsbSio = new LpcUsbSioI2c(config.portConfig, config.i2cConfig);
    }
    else if (config.function == LpcUsbSio::kFunction_GPIO)
    {
        // Currently, GPIO function is not supported.
        throw std::runtime_error(format_string("Error: GPIO function is not supported yet.\n"));
    }
    else
    {
        throw std::runtime_error(format_string("Error: Invalid function of LPC USB Serial I/O.\n"));
    }
}

// See LpcUsbSioPeripheral.h for documentation of this method.
LpcUsbSioPeripheral::~LpcUsbSioPeripheral()
{
    if (m_lpcUsbSio)
    {
        delete m_lpcUsbSio;
        m_lpcUsbSio = NULL;
    }
}

// See LpcUsbSioPeripheral.h for documentation of this method.
status_t LpcUsbSioPeripheral::read(uint8_t *buffer, uint32_t requestedBytes, uint32_t *actualBytes, uint32_t timeoutMs)
{
    assert(buffer);

    uint32_t count = 0;
    if (requestedBytes)
    {
        // Read the requested number of bytes.
        count = m_lpcUsbSio->read(buffer, requestedBytes);
    }

    if (actualBytes)
    {
        *actualBytes = count;
    }

    if (Log::getLogger()->getFilterLevel() == Logger::kDebug2)
    {
        // Log bytes read in hex
        Log::debug2("<");
        for (int i = 0; i < (int)count; i++)
        {
            Log::debug2("%02x", buffer[i]);
            if (i != (count - 1))
            {
                Log::debug2(" ");
            }
        }
        Log::debug2(">\n");
    }

    if (count == 0)
    {
        // Nothing got is a timeout error.
        return kStatus_Timeout;
    }
    else if (count != requestedBytes)
    {
        // Fewer than requested is a failure.
        return kStatus_Fail;
    }

    return kStatus_Success;
}

// See LpcUsbSioPeripheral.h for documentation of this method.
status_t LpcUsbSioPeripheral::write(const uint8_t *buffer, uint32_t byteCount)
{
    assert(buffer);

    if (Log::getLogger()->getFilterLevel() == Logger::kDebug2)
    {
        // Log bytes written in hex
        Log::debug2("[");
        for (int i = 0; i < (int)byteCount; i++)
        {
            Log::debug2("%02x", buffer[i]);
            if (i != (byteCount - 1))
            {
                Log::debug2(" ");
            }
        }
        Log::debug2("]\n");
    }

    if (byteCount == 0)
    {
        return kStatus_Success;
    }

    uint32_t count = m_lpcUsbSio->write(buffer, byteCount);

    if (count < byteCount)
    {
        // Fewer than requested is a failure.
        return kStatus_Fail;
    }

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
