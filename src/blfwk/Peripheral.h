/*
 * Copyright (c) 2013-2014 Freescale Semiconductor, Inc.
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _Peripheral_h_
#define _Peripheral_h_

#include <string.h>
#include "BusPal.h"
#include "LpcUsbSio.h"
#include "bootloader_common.h"

//! @addtogroup host_peripherals
//! @{

namespace blfwk
{
/*!
 * @brief Represents a peripheral.
 *
 * Interface class for objects that provide the source for commands or sink for responses.
 */
class Peripheral
{
public:
    enum _host_peripheral_types
    {
        kHostPeripheralType_None,
        kHostPeripheralType_UART,
        kHostPeripheralType_BUSPAL_UART,
        kHostPeripheralType_LPCUSBSIO,
        kHostPeripheralType_USB_HID,
        kHostPeripheralType_SIM,
        kHostPeripheralType_I2C,
        kHostPeripheralType_SPI,
    };

    struct PeripheralConfigData
    {
        _host_peripheral_types peripheralType;
        bool ping;
        std::string comPortName;
        long comPortSpeed;
        uint32_t packetTimeoutMs;
        unsigned short usbHidVid;
        unsigned short usbHidPid;
        std::string usbHidSerialNumber;
        std::string usbPath;
#if defined(LINUX) && defined(__ARM__)
        unsigned char i2cAddress;
        unsigned char spiPolarity;
        unsigned char spiPhase;
        unsigned char spiSequence;
#endif // #if defined(LINUX) && defined(__ARM__)
        BusPal::BusPalConfigData busPalConfig;
        LpcUsbSio::LpcUsbSioConfigData lpcUsbSioConfig;
    };

    virtual ~Peripheral(){};

    //! @brief Read bytes.
    //!
    //! @param buffer Pointer to buffer
    //! @param requestedBytes Number of bytes to read
    //! @param timeoutMs Time in milliseconds to wait for read to complete.
    //! @param actualBytes Number of bytes actually read.
    virtual status_t read(uint8_t *buffer, uint32_t requestedBytes, uint32_t *actualBytes, uint32_t timeout) = 0;

    //! @brief Write bytes.
    virtual status_t write(const uint8_t *buffer, uint32_t byteCount) = 0;

    //! @brief Return peripheral Type
    virtual _host_peripheral_types get_type(void) = 0;
};

} // namespace blfwk

//! @}

#endif // _Peripheral_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
