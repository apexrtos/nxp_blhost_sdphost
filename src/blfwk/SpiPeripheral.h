/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef _spi_peripheral_h_
#define _spi_peripheral_h_

#include "Peripheral.h"
#include "blfwk/spi.h"
#include "packet/command_packet.h"

//! @addtogroup spi_peripheral
//! @{

namespace blfwk
{
/*!
 * @brief Peripheral that talks to the target device over SPI port hardware.
 */
class SpiPeripheral : public Peripheral
{
public:
    //! @breif Constants.
    enum _spi_peripheral_constants
    {
        // The read() implementation for the SpiPeripheral does not use this the timeout parameter.
        kSpiPeripheral_UnusedTimeout = 0,
        // Serial timeout is set to this default during init().
        kSpiPeripheral_DefaultReadTimeoutMs = 1,
        kSpiPeripheral_DefaultSpeedKHz = 100,
        kSpiPeripheral_DefaultClockPolarity = 1,
        kSpiPeripheral_DefaultClockPhase = 1,
        kSpiPeripheral_DefaultBitSequence = 1,
        kSpiPeripheral_DefaultBitsPerWord = 8,
    };

public:
    //! @brief Parameterized constructor that opens the serial port.
    //!
    //! Opens and configures the port. Throws exception if port configuration fails.
    //!
    //! @param port OS file path for SPI port. For example "/dev/spidev-0.0" on Linux.
    //! @param speed Port speed in KHz, e.g. 100(means 100KHz).
    //! @param polarity SPI clock polarity, 1 for active-high, 0 for active-low.
    //! @param phase SPI clock phase, 1 for second clock edge, 0 for first clock edge.
    //! @param sequence SPI data transfer bits sequence. 1 for LSB, 0 for MSB.
    SpiPeripheral(const char *port,
                  long speed = kSpiPeripheral_DefaultSpeedKHz,
                  uint8_t polarity = kSpiPeripheral_DefaultClockPolarity,
                  uint8_t phase = kSpiPeripheral_DefaultClockPhase,
                  uint8_t sequence = kSpiPeripheral_DefaultBitSequence);

    //! @brief Destructor.
    virtual ~SpiPeripheral();

    //! @brief Flush.
    //!
    //! should be called on an open SPI port in order to flush any remaining data in the SPI RX buffer
    void flushRX();

    //! @brief Read bytes.
    //!
    //! @param buffer Pointer to buffer.
    //! @param requestedBytes Number of bytes to read.
    //! @param actualBytes Number of bytes actually read.
    //! @param timeoutMs Time in milliseconds to wait for read to complete.
    virtual status_t read(uint8_t *buffer, uint32_t requestedBytes, uint32_t *actualBytes, uint32_t unused_timeoutMs);

    //! @brief Write bytes.
    //!
    //! @param buffer Pointer to buffer to write
    //! @param byteCount Number of bytes to write
    virtual status_t write(const uint8_t *buffer, uint32_t byteCount);

    //! @brief Return peripheral Type
    virtual _host_peripheral_types get_type(void) { return kHostPeripheralType_SPI; }

protected:
    //! @brief Initialize.
    //!
    //! Opens and configures the port.
    //!
    //! Note: following COM port configuration is assumed: 8 bits, 1 stop bit, no parity.
    //!
    //! @param port OS file path for SPI port. For example "/dev/spidev-0.0" on Linux.
    //! @param speed Port speed in KHz, e.g. 100(means 100KHz).
    //! @param polarity SPI clock polarity, 1 for active-high, 0 for active-low.
    //! @param phase SPI clock phase, 1 for second clock edge, 0 for first clock edge.
    //! @param direction SPI data transfer bits direction. 1 for LSB, 0 for MSB.
    bool init(const char *port, long speed, uint8_t polarity, uint8_t phase, uint8_t direction);

    int m_fileDescriptor;                    //!< Port file descriptor.
    uint8_t m_buffer[kDefaultMaxPacketSize]; //!< Buffer for bytes used to build read packet.
    uint32_t m_current_ReadTimeout;          //!< The last value sent to serial_set_read_timeout().
};

} // namespace blfwk

//! @}

#endif // _spi_peripheral_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
