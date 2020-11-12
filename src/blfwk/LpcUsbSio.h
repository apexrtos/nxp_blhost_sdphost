/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef _LpcUsbSio_h_
#define _LpcUsbSio_h_

#include <string.h>
#include "host_types.h"
#include "middleware/lpcusbsio/inc/lpcusbsio.h"

//! @addtogroup lpc_usb_sio
//! @{

namespace blfwk
{
/*!
 * @brief Interface with the LPC USB Serial I/O
 */
class LpcUsbSio
{
public:
    //! @brief Constants

    enum _lpc_usb_sio_port_contants
    {
        kDefault_Vid = LPCUSBSIO_VID, //!< NXP VID
        kDefault_Pid = LPCUSBSIO_PID, //!< PID for LPC USB Serial I/O
    };

    enum _lpc_usb_sio_timeout_contants
    {
        kDefault_TimeoutMs = LPCUSBSIO_READ_TMO, //!< 500ms, timeout miliseconds for reading
    };

    enum lpc_usb_sio_function_t
    {
        kFunction_None,
        kFunction_SPI,
        kFunction_I2C,
        kFunction_GPIO,
    };
    //! @brief SPI clock polarity configuration.
    enum spi_clock_polarity_t
    {
        kSpiPolarity_ActiveHigh = 0, //!< Active-high SPI clock (idles low).
        kSpiPolarity_ActiveLow = 1   //!< Active-low SPI clock (idles high).
    };

    //! @brief SPI clock phase configuration.
    enum spi_clock_phase_t
    {
        kSpiPhase_FirstEdge = 0, //!< First edge on SPSCK occurs at the middle of the first cycle of a data transfer.
        kSpiPhase_SecondEdge = 1 //!< First edge on SPSCK occurs at the start of the first cycle of a data transfer.
    };

    //! @brief I2C slave address
    enum i2c_slave_address_t
    {
        kI2cAddress_Default = 0x10, //!< Default I2C slave address.
    };

    //! @brief LPC USB Serial I/O USB port configuration.
    struct LpcUsbSioPortConfigData
    {
        uint16_t usbVid;
        uint16_t usbPid;
        std::string usbString;
        std::string usbPath;
        int32_t portIndex;

        LpcUsbSioPortConfigData()
        {
            usbVid = LpcUsbSio::kDefault_Vid;
            usbPid = LpcUsbSio::kDefault_Pid;
            portIndex = 0;
        }
    };

    //! @brief LPC USB Serial I/O SPI configuration.
    struct LpcUsbSioSpiConfigData
    {
        uint32_t spiSpeedHz;
        spi_clock_polarity_t spiPolarity;
        spi_clock_phase_t spiPhase;
        uint8_t spiSselPort;
        uint8_t spiSselPin;
        int32_t spiPortIndex;

        LpcUsbSioSpiConfigData()
        {
            spiSpeedHz = 100000;
            spiPolarity = LpcUsbSio::kSpiPolarity_ActiveLow;
            spiPhase = LpcUsbSio::kSpiPhase_SecondEdge;
            spiSselPort = 0;
            spiSselPin = 0;
            spiPortIndex = 0;
        }
    };

    //! @brief LPC USB Serial I/O I2C configuration.
    struct LpcUsbSioI2cConfigData
    {
        uint8_t i2cAddress;
        I2C_CLOCKRATE_T i2cSpeedHz;
        int32_t i2cPortIndex;

        LpcUsbSioI2cConfigData()
        {
            i2cAddress = LpcUsbSio::kI2cAddress_Default;
            i2cSpeedHz = I2C_CLOCK_STANDARD_MODE;
            i2cPortIndex = 0;
        }
    };

    //! @brief LPC USB Serial I/O GPIO configuration.
    struct LpcUsbSioGpioConfigData
    {
        uint8_t gpioPort;
        uint8_t gpioPin;
        uint8_t gpioValue;
        uint8_t gpioMode;
        int32_t gpioPortIndex;

        LpcUsbSioGpioConfigData()
        {
            gpioPort = 0;
            gpioPin = 0;
            gpioValue = 0;
            gpioMode = 0;
            gpioPortIndex = 0;
        }
    };

    //! @brief LPC USB Serial I/O configuration data.
    struct LpcUsbSioConfigData
    {
        lpc_usb_sio_function_t function;
        LpcUsbSioPortConfigData portConfig;
        LpcUsbSioSpiConfigData spiConfig;
        LpcUsbSioI2cConfigData i2cConfig;

        LpcUsbSioConfigData() { function = LpcUsbSio::kFunction_None; }
    };

    //! @brief Constructor.
    LpcUsbSio(const LpcUsbSio::LpcUsbSioPortConfigData &config);

    //! @brief Destructor.
    virtual ~LpcUsbSio();

    //! @brief Parse the params in options into the config structure.
    static bool parse(const string_vector_t &params, LpcUsbSio::LpcUsbSioConfigData &config);

    //! @brief Convert LPC USB Serial I/O error code to detail string.
    static const std::string getError(LPCUSBSIO_ERR_T errorCode);

    //! @brief Write data.
    virtual uint32_t write(const uint8_t *data, uint32_t byteCount) = 0;

    //! @brief Read data.
    virtual uint32_t read(uint8_t *data, uint32_t byteCount) = 0;

protected:
    uint16_t m_usbVid;        //!< The VID of LPC USB Serial I/O.
    uint16_t m_usbPid;        //!< The PID of LPC USB Serial I/O.
    std::string m_usbString;  //!< The USB Serial String of LPC USB Serial I/O port.
    std::string m_usbPath;    //!< The USB instance path of LPC USB Serial I/O port.
    int32_t m_portCount;      //!< The number of LPC USB Serial I/O connected.
    int32_t m_portIndex;      //!< The port of LPC USB Serial I/O which is active. Index starts from 0.
    LPC_HANDLE m_portHandler; //!< The LPC USB Serial I/O hardware operation handler.
};

class LpcUsbSioSpi : public LpcUsbSio
{
public:
    //! @brief Constructor.
    LpcUsbSioSpi(const LpcUsbSio::LpcUsbSioPortConfigData &portConfig,
                 const LpcUsbSio::LpcUsbSioSpiConfigData &spiConfig);

    //! @brief Destructor.
    virtual ~LpcUsbSioSpi();

    //! @brief Write data.
    virtual uint32_t write(const uint8_t *data, uint32_t byteCount);

    //! @brief Read data.
    virtual uint32_t read(uint8_t *data, uint32_t byteCount);

protected:
    int32_t m_spiPortCount;      //!< The number of SPI port on current LPC USB Serial I/O.
    int32_t m_spiPortIndex;      //!< The port of SPI which is active. Index starts from 0.
    uint8_t m_spiSselPort;       //!< The port numer of the SPI->SSELn pin on LPC USB Serial I/O.
    uint8_t m_spiSselPin;        //!< The pin number of the SPI->SSELn pin on LPC USB Serial I/O.
    LPC_HANDLE m_spiPortHandler; //!< The SPI operation handler.
};

class LpcUsbSioI2c : public LpcUsbSio
{
public:
    //! @brief Constructor.
    LpcUsbSioI2c(const LpcUsbSio::LpcUsbSioPortConfigData &portConfig,
                 const LpcUsbSio::LpcUsbSioI2cConfigData &i2cConfig);

    //! @brief Destructor.
    virtual ~LpcUsbSioI2c();

    //! @brief Write data.
    virtual uint32_t write(const uint8_t *data, uint32_t byteCount);

    //! @brief Read data.
    virtual uint32_t read(uint8_t *data, uint32_t byteCount);

protected:
    int32_t m_i2cPortCount;      //!< The number of I2C port on current LPC USB Serial I/O.
    int32_t m_i2cPortIndex;      //!< The port of I2C which is active. Index starts from 0.
    LPC_HANDLE m_i2cPortHandler; //!< The I2C operation handler.
    uint8_t m_i2cAddress;        //!< The I2C slave address to communicate.
};

} // namespace blfwk

//! @}

#endif // _LpcUsbSio_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
