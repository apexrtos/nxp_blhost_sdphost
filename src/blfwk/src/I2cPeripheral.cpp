/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "blfwk/I2cPeripheral.h"
#include "blfwk/Logging.h"
#include "blfwk/format_string.h"
#include "blfwk/i2c.h"

using namespace blfwk;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See I2cPeripheral.h for documentation of this method.
I2cPeripheral::I2cPeripheral(const char *port, long speed, uint8_t address)
    : m_fileDescriptor(-1)
{
    if (!init(port, address, speed))
    {
        throw std::runtime_error(format_string("Error: Cannot open I2C port(%s), speed(%d Hz).\n", port, speed));
    }
}

// See I2cPeripheral.h for documentation of this method.
bool I2cPeripheral::init(const char *port, long speed, uint8_t address)
{
    assert(port);

    // Open the port.
    m_fileDescriptor = i2c_open(const_cast<char *>(port));
    if (m_fileDescriptor == -1)
    {
        return false;
    }

    i2c_setup(m_fileDescriptor, speed, address);

    // Flush garbage from receive buffer before setting read timeout.
    flushRX();

    // Set host serial timeout to 10 milliseconds. Higherlevel timeouts are implemented in
    // SerialPacketizer.cpp
    i2c_set_timeout(m_fileDescriptor, kI2cPeripheral_DefaultReadTimeoutMs);

    return true;
}

// See I2cPeripheral.h for documentation of this method.
I2cPeripheral::~I2cPeripheral()
{
    if (m_fileDescriptor != -1)
    {
        i2c_close(m_fileDescriptor);
    }
}

// See I2cPeripheral.h for documentation of this method.
status_t I2cPeripheral::read(uint8_t *buffer, uint32_t requestedBytes, uint32_t *actualBytes, uint32_t unused_timeoutMs)
{
    assert(buffer);

    // Read the requested number of bytes.
    int count = i2c_read(m_fileDescriptor, reinterpret_cast<char *>(buffer), requestedBytes);
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

    if (count < (int)requestedBytes)
    {
        // Anything less than requestedBytes is a timeout error.
        return kStatus_Timeout;
    }

    return kStatus_Success;
}

// See I2cPeripheral.h for documentation of this method.
void I2cPeripheral::flushRX() {}

// See I2cPeripheral.h for documentation of this method.
status_t I2cPeripheral::write(const uint8_t *buffer, uint32_t byteCount)
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

    if (i2c_write(m_fileDescriptor, reinterpret_cast<char *>(const_cast<uint8_t *>(buffer)), byteCount) == byteCount)
        return kStatus_Success;
    else
        return kStatus_Fail;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
