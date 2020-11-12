/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "blfwk/Logging.h"
#include "blfwk/SpiPeripheral.h"
#include "blfwk/format_string.h"
#include "blfwk/spi.h"

using namespace blfwk;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See SpiPeripheral.h for documentation of this method.
SpiPeripheral::SpiPeripheral(const char *port, long speed, uint8_t polarity, uint8_t phase, uint8_t sequence)
    : m_fileDescriptor(-1)
{
    if (!init(port, speed, polarity, phase, sequence))
    {
        throw std::runtime_error(
            format_string("Error: Cannot open SPI port(%s), speed(%d Hz), polarity(%d), phase(%d), %s.\n", port, speed,
                          polarity, phase, sequence ? "LSB" : "MSB"));
    }
}

// See SpiPeripheral.h for documentation of this method.
bool SpiPeripheral::init(const char *port, long speed, uint8_t polarity, uint8_t phase, uint8_t sequence)
{
    assert(port);

    // Open the port.
    m_fileDescriptor = spi_open(const_cast<char *>(port));
    if (m_fileDescriptor == -1)
    {
        return false;
    }

    uint32_t mode = 0x0;
    if (polarity == 0)
    {
        mode &= ~SPI_CPOL;
    }
    else if (polarity == 1)
    {
        mode |= SPI_CPOL;
    }
    else
    {
        return false;
    }

    if (phase == 0)
    {
        mode &= ~SPI_CPHA;
    }
    else if (phase == 1)
    {
        mode |= SPI_CPHA;
    }
    else
    {
        return false;
    }

    if (sequence == 0)
    {
        mode &= ~SPI_LSB_FIRST;
    }
    else if (sequence == 1)
    {
        mode |= SPI_LSB_FIRST;
    }
    else
    {
        return false;
    }

    spi_setup(m_fileDescriptor, speed * 1000, mode, kSpiPeripheral_DefaultBitsPerWord);

    // Flush garbage from receive buffer before setting read timeout.
    flushRX();

    // Set host serial timeout to 10 milliseconds. Higherlevel timeouts are implemented in
    // SerialPacketizer.cpp
    spi_set_timeout(m_fileDescriptor, kSpiPeripheral_DefaultReadTimeoutMs);

    return true;
}

// See SpiPeripheral.h for documentation of this method.
SpiPeripheral::~SpiPeripheral()
{
    if (m_fileDescriptor != -1)
    {
        spi_close(m_fileDescriptor);
    }
}

// See SpiPeripheral.h for documentation of this method.
status_t SpiPeripheral::read(uint8_t *buffer, uint32_t requestedBytes, uint32_t *actualBytes, uint32_t unused_timeoutMs)
{
    assert(buffer);

    // Read the requested number of bytes.
    int count = spi_read(m_fileDescriptor, reinterpret_cast<char *>(buffer), requestedBytes);
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

// See SpiPeripheral.h for documentation of this method.
void SpiPeripheral::flushRX() {}

// See SpiPeripheral.h for documentation of this method.
status_t SpiPeripheral::write(const uint8_t *buffer, uint32_t byteCount)
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

    if (spi_write(m_fileDescriptor, reinterpret_cast<char *>(const_cast<uint8_t *>(buffer)), byteCount) == byteCount)
        return kStatus_Success;
    else
        return kStatus_Fail;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
