/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef _LpcUsbSioPeripheral_h_
#define _LpcUsbSioPeripheral_h_

#include "Peripheral.h"

//! @addtogroup lpc_usb_sio_peripheral
//! @{

namespace blfwk
{
/*!
 * @brief Peripheral that talks to the target device over LPCUSB Serial I/O hardware.
 */
class LpcUsbSioPeripheral : public Peripheral
{
public:
    //! @brief Parameterized constructor that opens the LPC USB Serial I/O port.
    //!
    //! Opens and configures the port. Throws exception if operation fails.
    //!
    //! @param config The configuration data of LPCUSB Serial I/O device.
    LpcUsbSioPeripheral(const LpcUsbSio::LpcUsbSioConfigData &config);

    //! @brief Destructor.
    virtual ~LpcUsbSioPeripheral();

    //! @brief Read bytes.
    //!
    //! @param buffer Pointer to buffer
    //! @param requestedBytes Number of bytes to read
    //! @param actualBytes Number of bytes got
    //! @param timeoutMs The timeout microseconds for the read operation
    virtual status_t read(uint8_t *buffer, uint32_t requestedBytes, uint32_t *actualBytes, uint32_t timeoutMs);

    //! @brief Write bytes.
    //!
    //! @param buffer Pointer to buffer to write
    //! @param byteCount Number of bytes to write
    virtual status_t write(const uint8_t *buffer, uint32_t byteCount);

    //! @brief Get the peripheral type.
    virtual _host_peripheral_types get_type(void) { return kHostPeripheralType_LPCUSBSIO; }

protected:
    LpcUsbSio *m_lpcUsbSio; //!< Represents LPC USB Serial I/O hardware.
};

} // namespace blfwk

//! @}

#endif // _LpcUsbSioPeripheral_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
