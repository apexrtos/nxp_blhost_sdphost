/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef _i2c_peripheral_h_
#define _i2c_peripheral_h_

#include "Peripheral.h"
#include "blfwk/i2c.h"
#include "packet/command_packet.h"

//! @addtogroup i2c_peripheral
//! @{

namespace blfwk
{
/*!
 * @brief Peripheral that talks to the target device over I2C port hardware.
 */
class I2cPeripheral : public Peripheral
{
public:
    //! @breif Constants.
    enum _i2c_peripheral_constants
    {
        // The read() implementation for the I2cPeripheral does not use this the timeout parameter.
        kI2cPeripheral_UnusedTimeout = 0,
        // Serial timeout is set to this default during init().
        kI2cPeripheral_DefaultReadTimeoutMs = 10,
        kI2cPeripheral_DefaultSpeedKHz = 100,
        kI2cPeripheral_DefaultAddress = 0x10
    };

public:
    //! @brief Parameterized constructor that opens the serial port.
    //!
    //! Opens and configures the port. Throws exception if port configuration fails.
    //!
    //! @param port OS file path for I2C port. For example "/dev/i2c-0.0" on Linux.
    //! @param speed Port speed, e.g. 9600.
    //! @param address I2C slave's address
    I2cPeripheral(const char *port,
                  long speed = kI2cPeripheral_DefaultSpeedKHz,
                  uint8_t address = kI2cPeripheral_DefaultAddress);

    //! @brief Destructor.
    virtual ~I2cPeripheral();

    //! @brief Flush.
    //!
    //! should be called on an open I2C port in order to flush any remaining data in the I2C RX buffer
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
    virtual _host_peripheral_types get_type(void) { return kHostPeripheralType_I2C; }

protected:
    //! @brief Initialize.
    //!
    //! Opens and configures the port.
    //!
    //! Note: following COM port configuration is assumed: 8 bits, 1 stop bit, no parity.
    //!
    //! @param port OS file path for I2C port. For example "/dev/i2c-0.0" on Linux.
    //! @param speed Port speed, e.g. 9600.
    //! @param address I2C slave's address
    bool init(const char *port, long speed, uint8_t address);

    int m_fileDescriptor;                    //!< Port file descriptor.
    uint8_t m_buffer[kDefaultMaxPacketSize]; //!< Buffer for bytes used to build read packet.
    uint32_t m_current_ReadTimeout;          //!< The last value sent to serial_set_read_timeout().
};

} // namespace blfwk

//! @}

#endif // _i2c_peripheral_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
