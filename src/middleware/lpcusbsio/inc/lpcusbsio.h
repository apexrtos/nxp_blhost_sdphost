/*
 * @brief LPC USB serial I/O interface definition
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */
#ifndef __LPCUSBSIO_H
#define __LPCUSBSIO_H

#include <stdint.h>
#include "lpcusbsio_protocol.h"

#if defined(LPCUSBSIO_EXPORTS)
#define LPCUSBSIO_API __declspec(dllexport)
#elif defined(LPCUSBSIO_IMPORTS)
#define LPCUSBSIO_API __declspec(dllimport)
#else
#define LPCUSBSIO_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup LPCUSBSIO_API LPC USB serial I/O (LPCUSBSIO) API interface
 * <b>API description</b><br>
 * The LPCUSBSIO APIs can be divided into two broad sets. The first set consists of
 * five control APIs and the second set consists of two data transferring APIs. On error
 * most APIs return an LPCUSBSIO_ERR_T code. Application code can call LPCUSBSIO_Error() routine
 * to get user presentable uni-code string corresponding to the last error.
 * <br>
 * The current version of LPCUSBSIO allows communicating with I2C, SPI slave devices and GPIO.
 *
 * @{
 */

/** NXP USB-IF vendor ID. */
#define LPCUSBSIO_VID                       0x1FC9
/** USB-IF product ID for LPCUSBSIO devices. */
#define LPCUSBSIO_PID                       0x0090
/** Read time-out value in milliseconds used by the library. If a response is not received
 *
 */
#define LPCUSBSIO_READ_TMO                  500

/** I2C_IO_OPTIONS Options to I2C_DeviceWrite & I2C_DeviceRead routines
* @{
*/
/** Generate start condition before transmitting */
#define I2C_TRANSFER_OPTIONS_START_BIT      0x0001

/** Generate stop condition at the end of transfer */
#define I2C_TRANSFER_OPTIONS_STOP_BIT       0x0002

/** Continue transmitting data in bulk without caring about Ack or nAck from device if this bit is
*  not set. If this bit is set then stop transmitting the data in the buffer when the device nAcks
*/
#define I2C_TRANSFER_OPTIONS_BREAK_ON_NACK  0x0004

/** lpcusbsio-I2C generates an ACKs for every Byte read. Some I2C slaves require the I2C
master to generate a nACK for the last data Byte read. Setting this bit enables working with such
I2C slaves */
#define I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE 0x0008

/* Setting this bit would mean that the address field should be ignored.
* The address is either a part of the data or this is a special I2C
* frame that doesn't require an address. For example when transferring a
* frame greater than the USB_HID packet this option can be used.
*/
#define I2C_TRANSFER_OPTIONS_NO_ADDRESS     0x00000040

/** @} */

/** I2C_FAST_TRANSFER_OPTIONS I2C master faster transfer options
* @{
*/

/** Ignore NACK during data transfer. By default transfer is aborted. */
#define I2C_FAST_XFER_OPTION_IGNORE_NACK     0x01
/** ACK last Byte received. By default we NACK last Byte we receive per I2C specification. */
#define I2C_FAST_XFER_OPTION_LAST_RX_ACK     0x02

/**
* @}
*/

/******************************************************************************
*								Type defines
******************************************************************************/
/** @brief Handle type */
typedef void *LPC_HANDLE;

/** @brief Error types returned by LPCUSBSIO APIs */
typedef enum LPCUSBSIO_ERR_t {
	/** All API return positive number for success */
	LPCUSBSIO_OK = 0,
	/** HID library error. */
	LPCUSBSIO_ERR_HID_LIB = -1,
	/** Handle passed to the function is invalid. */
	LPCUSBSIO_ERR_BAD_HANDLE = -2,
	/** Thread Synchronization error. */
	LPCUSBSIO_ERR_SYNCHRONIZATION = -3,
	/** Memory Allocation error. */
	LPCUSBSIO_ERR_MEM_ALLOC = -4,
	/** Mutex Creation error. */
	LPCUSBSIO_ERR_MUTEX_CREATE = -5,

	/* Errors from hardware I2C interface*/
	/** Fatal error occurred */
	LPCUSBSIO_ERR_FATAL = -0x11,
	/** Transfer aborted due to NACK  */
	LPCUSBSIO_ERR_I2C_NAK = -0x12,
	/** Transfer aborted due to bus error  */
	LPCUSBSIO_ERR_I2C_BUS = -0x13,
	/** NAK received after SLA+W or SLA+R  */
	LPCUSBSIO_ERR_I2C_SLAVE_NAK = -0x14,
	/** I2C bus arbitration lost to other master  */
	LPCUSBSIO_ERR_I2C_ARBLOST = -0x15,

	/* Errors from firmware's HID-SIO bridge module */
	/** Transaction timed out */
	LPCUSBSIO_ERR_TIMEOUT = -0x20,
	/** Invalid HID_SIO Request or Request not supported in this version. */
	LPCUSBSIO_ERR_INVALID_CMD = -0x21,
	/** Invalid parameters are provided for the given Request. */
	LPCUSBSIO_ERR_INVALID_PARAM = -0x22,
	/** Partial transfer completed. */
	LPCUSBSIO_ERR_PARTIAL_DATA = -0x23,
} LPCUSBSIO_ERR_T;

/** @brief I2C clock rates */
typedef enum I2C_ClockRate_t {
	I2C_CLOCK_STANDARD_MODE = 100000,							/*!< 100kb/sec */
	I2C_CLOCK_FAST_MODE = 400000,								/*!< 400kb/sec */
	I2C_CLOCK_FAST_MODE_PLUS = 1000000,							/*!< 1000kb/sec */
} I2C_CLOCKRATE_T;

/** @brief I2C Port configuration information */
typedef struct PortConfig_t {
	I2C_CLOCKRATE_T ClockRate;									/*!< I2C Clock speed */
	uint32_t          Options;									/*!< Configuration options */
} I2C_PORTCONFIG_T;

/** @brief I2C Fast transfer parameter structure */
typedef struct I2CFastXferParam_t {
	uint16_t txSz;				/*!< Number of bytes in transmit array,
								   if 0 only receive transfer will be carried on */
	uint16_t rxSz;				/*!< Number of bytes to received,
								   if 0 only transmission we be carried on */
	uint16_t options;			/*!< Fast transfer options */
	uint16_t slaveAddr;			/*!< 7-bit I2C Slave address */
	const uint8_t *txBuff;		/*!< Pointer to array of bytes to be transmitted */
	uint8_t *rxBuff;			/*!< Pointer memory where bytes received from I2C be stored */
} I2C_FAST_XFER_T;

/** Macro to generate SPI device number from port and pin */
#define LPCUSBSIO_GEN_SPI_DEVICE_NUM(port, pin) ((((uint8_t)(port) & 0x07) << 5) | ((pin) & 0x1F))

/** @brief SPI transfer parameter structure */
typedef struct SPIXferParam_t {
	uint16_t length;				/*!< Number of bytes to transmit and receive */
	uint8_t options;			/*!< Transfer options */
	uint8_t device;				/*!< SPI slave device, use @ref LPCUSBSIO_GEN_SPI_DEVICE_NUM macro
								to derive device number from a GPIO port and pin number */
	const uint8_t *txBuff;		/*!< Pointer to array of bytes to be transmitted */
	uint8_t *rxBuff;			/*!< Pointer memory where bytes received from SPI be stored */
} SPI_XFER_T;

/******************************************************************************
*								LPCUSBSIO functions
******************************************************************************/

/** @brief Get number LPCUSBSIO ports available on the LPC controller.
 *
 * This function gets the number of LPCUSBSIO ports that are available on the LPC controller.
 * The number of ports available in each of these chips is different.
 *
 * @param vid		: Vendor ID.
 * @param pid		: Product ID.
 *
 * @returns
 * The number of ports available on the LPC controller.
 *
 */
LPCUSBSIO_API int LPCUSBSIO_GetNumPorts(uint32_t vid, uint32_t pid);

/** @brief Opens the indexed Serial IO port.
*
* This function opens the indexed port and provides a handle to it. Valid values for
* the index of port can be from 0 to the value obtained using LPCUSBSIO_GetNumPorts
* – 1).
*
* @param index		: Index of the port to be opened.
*
* @returns
* This function returns a handle to LPCUSBSIO port object on
* success or NULL on failure.
*/
LPCUSBSIO_API LPC_HANDLE LPCUSBSIO_Open(uint32_t index);


/** @brief Closes a LPC Serial IO port.
*
* Closes a Serial IO port and frees all resources that were used by it.
*
* @param hUsbSio	: Handle of the LPSUSBSIO port.
*
* @returns
* This function returns LPCUSBSIO_OK on success and negative error code on failure.
* Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t LPCUSBSIO_Close(LPC_HANDLE hUsbSio);

/** @brief Get version string of the LPCUSBSIO library.
*
* @param hUsbSio	: A device handle returned from LPCUSBSIO_Open().
*
* @returns
* This function returns a string containing the version of the library.
* If the device handle passed is not NULL then the firmware version of
* the connected device is appended to the string.
*/
LPCUSBSIO_API const char *LPCUSBSIO_GetVersion(LPC_HANDLE hUsbSio);

/** @brief Get a string describing the last error which occurred.
*
* @param hUsbSio	: A device handle returned from LPCUSBSIO_Open().
*
* @returns
* This function returns a string containing the last error
* which occurred or NULL if none has occurred.
*
*/
LPCUSBSIO_API const wchar_t *LPCUSBSIO_Error(LPC_HANDLE hUsbSio);

/** @brief Returns the number of I2C ports supported by Serial IO device.
*
* @param hUsbSio	: A device handle returned from LPCUSBSIO_Open().
*
* @returns
* This function returns the number of I2C ports on success and negative error code on failure.
* Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API uint32_t LPCUSBSIO_GetNumI2CPorts(LPC_HANDLE hUsbSio);

/** @brief Returns the number of SPI ports supported by Serial IO device.
*
* @param hUsbSio	: A device handle returned from LPCUSBSIO_Open().
*
* @returns
* This function returns the number of SPI ports on success and negative error code on failure.
* Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API uint32_t LPCUSBSIO_GetNumSPIPorts(LPC_HANDLE hUsbSio);

/** @brief Returns the number of GPIO ports supported by Serial IO device.
*
* @param hUsbSio	: A device handle returned from LPCUSBSIO_Open().
*
* @returns
* This function returns the number of GPIO ports on success and negative error code on failure.
* Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API uint32_t LPCUSBSIO_GetNumGPIOPorts(LPC_HANDLE hUsbSio);

/** @brief Returns the max number of bytes supported for I2C/SPI transfers by the Serial IO device.
*
* @param hUsbSio	: A device handle returned from LPCUSBSIO_Open().
*
* @returns
* This function returns the max data transfer size on success and negative error code on failure.
* Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API uint32_t LPCUSBSIO_GetMaxDataSize(LPC_HANDLE hUsbSio);

/** @brief Returns the last error seen by the Library.
*
* @returns
* This function returns the last error seen by the library.
* Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t LPCUSBSIO_GetLastError(void);

/******************************************************************************
*								I2C functions
******************************************************************************/

/** @brief Initialize a I2C port.
 *
 * This function initializes the I2C port and the communication parameters associated
 * with it.
 *
 * @param hUsbSio	: Handle of the LPSUSBSIO port.
 * @param config	: Pointer to I2C_PORTCONFIG_T structure. Members of
 * I2C_PORTCONFIG_T structure contains the values for I2C
 * master clock and Options
 * @param portNum		: I2C port number.
 *
 * @returns
 * This function returns a handle to I2C port object on success or NULL on failure.
 * Use LPCUSBSIO_Error() function to get last error.
 */
LPCUSBSIO_API LPC_HANDLE I2C_Open(LPC_HANDLE hUsbSio, I2C_PORTCONFIG_T *config, uint8_t portNum);

/** @brief Closes a I2C port.
 *
 * Deinitializes I2C port and frees all resources that were used by it.
 *
 * @param hI2C	: Handle of the I2C port.
 *
 * @returns
 * This function returns LPCUSBSIO_OK on success and negative error code on failure.
 * Check @ref LPCUSBSIO_ERR_T for more details on error code.
 *
 */
LPCUSBSIO_API int32_t I2C_Close(LPC_HANDLE hI2C);

/** @brief Reset I2C Controller.
 *
 *  @param hI2C	: A device handle returned from I2C_Open().
 *
 *  @returns
 *  This function returns LPCUSBSIO_OK on success and negative error code on failure.
 *  Check @ref LPCUSBSIO_ERR_T for more details on error code.
 *
 */
LPCUSBSIO_API int32_t I2C_Reset(LPC_HANDLE hI2C);


/** @brief Read from an addressed I2C slave.
 *
 * This function reads the specified number of bytes from an addressed I2C slave.
 * The @a options parameter effects the transfers. Some example transfers are shown below :
 * - When I2C_TRANSFER_OPTIONS_START_BIT, I2C_TRANSFER_OPTIONS_STOP_BIT and
 * I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE are set.
 *
 *  <b> S Addr Rd [A] [rxBuff0] A [rxBuff1] A ...[rxBuffN] NA P </b>
 *
 * - If I2C_TRANSFER_OPTIONS_NO_ADDRESS is also set.
 *
 *	<b> S [rxBuff0] A [rxBuff1] A ...[rxBuffN] NA P </b>
 *
 * - if I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE is not set
 *
 *  <b> S Addr Rd [A] [rxBuff0] A [rxBuff1] A ...[rxBuffN] A P </b>
 *
 * - If I2C_TRANSFER_OPTIONS_STOP_BIT is not set.
 *
 *  <b> S Addr Rd [A] [rxBuff0] A [rxBuff1] A ...[rxBuffN] NA </b>
 *
 * @param hI2C			: Handle of the I2C port.
 * @param deviceAddress	: Address of the I2C slave. This is a 7bit value and
 * it should not contain the data direction bit, i.e. the decimal
 * value passed should be always less than 128
 * @param buffer		: Pointer to the buffer where the read data is to be stored
 * @param sizeToTransfer: Number of bytes to be read
 * @param options: This parameter specifies data transfer options. Check HID_I2C_TRANSFER_OPTIONS_ macros.
 * @returns
 * This function returns number of bytes read on success and negative error code on failure.
 * Check @ref LPCUSBSIO_ERR_T for more details on error code.
 */
LPCUSBSIO_API int32_t I2C_DeviceRead(LPC_HANDLE hI2C,
									 uint8_t deviceAddress,
									 uint8_t *buffer,
									 uint16_t sizeToTransfer,
									 uint8_t options);

/** @brief Writes to the addressed I2C slave.
 *
 * This function writes the specified number of bytes to an addressed I2C slave.
 * The @a options parameter effects the transfers. Some example transfers are shown below :
 * - When I2C_TRANSFER_OPTIONS_START_BIT, I2C_TRANSFER_OPTIONS_STOP_BIT and
 * I2C_TRANSFER_OPTIONS_BREAK_ON_NACK are set.
 *
 *  <b> S Addr Wr[A] txBuff0[A] txBuff1[A] ... txBuffN[A] P </b>
 *
 *  - If I2C_TRANSFER_OPTIONS_NO_ADDRESS is also set.
 *
 *		<b> S txBuff0[A ] ... txBuffN[A] P </b>
 *
 *  - if I2C_TRANSFER_OPTIONS_BREAK_ON_NACK is not set
 *
 *      <b> S Addr Wr[A] txBuff0[A or NA] ... txBuffN[A or NA] P </b>
 *
 *  - If I2C_TRANSFER_OPTIONS_STOP_BIT is not set.
 *
 *      <b> S Addr Wr[A] txBuff0[A] txBuff1[A] ... txBuffN[A] </b>
 *
 * @param hI2C			: Handle of the I2C port.
 * @param deviceAddress	: Address of the I2C slave. This is a 7bit value and
 * it should not contain the data direction bit, i.e. the decimal
 * value passed should be always less than 128
 * @param buffer		: Pointer to the buffer where the data to be written is stored
 * @param sizeToTransfer: Number of bytes to be written
 * @param options		: This parameter specifies data transfer options. Check HID_I2C_TRANSFER_OPTIONS_ macros.
 * @returns
 * This function returns number of bytes written on success and negative error code on failure.
 * Check @ref LPCUSBSIO_ERR_T for more details on error code.
 */
LPCUSBSIO_API int32_t I2C_DeviceWrite(LPC_HANDLE hI2C,
									  uint8_t deviceAddress,
									  uint8_t *buffer,
									  uint16_t sizeToTransfer,
									  uint8_t options);

/**@brief	Transmit and Receive data in I2C master mode
 *
 * The parameter @a xfer should have its member @a slaveAddr initialized
 * to the 7 - Bit slave address to which the master will do the xfer, Bit0
 * to bit6 should have the address and Bit8 is ignored.During the transfer
 * no code(like event handler) must change the content of the memory
 * pointed to by @a xfer.The member of @a xfer, @a txBuff and @a txSz be
 * initialized to the memory from which the I2C must pick the data to be
 * transferred to slave and the number of bytes to send respectively, similarly
 * @a rxBuff and @a rxSz must have pointer to memory where data received
 * from slave be stored and the number of data to get from slave respectively.
 *
 * Following types of transfers are possible :
 * - Write-only transfer : When @a rxSz member of @a xfer is set to 0.
 *
 *     <b> S Addr Wr[A] txBuff0[A] txBuff1[A] ... txBuffN[A] P </b>
 *
 *  - If I2C_FAST_XFER_OPTION_IGNORE_NACK is set in @a options member
 *
 *     <b> S Addr Wr[A] txBuff0[A or NA] ... txBuffN[A or NA] P </b>
 *
 * - Read-only transfer : When @a txSz member of @a xfer is set to 0.
 *
 *      <b> S Addr Rd[A][rxBuff0] A[rxBuff1] A ...[rxBuffN] NA P </b>
 *
 *  - If I2C_FAST_XFER_OPTION_LAST_RX_ACK is set in @a options member
 *
 *      <b> S Addr Rd[A][rxBuff0] A[rxBuff1] A ...[rxBuffN] A P </b>
 *
 * - Read-Write transfer : When @a rxSz and @ txSz members of @a xfer are non - zero.
 *
 *      <b> S Addr Wr[A] txBuff0[A] txBuff1[A] ... txBuffN[A] <br>
 *          S Addr Rd[A][rxBuff0] A[rxBuff1] A ...[rxBuffN] NA P </b>
 *
 * @param hI2C	: Handle of the I2C port.
 * @param xfer	: Pointer to a I2C_FAST_XFER_T structure.
 * @returns
 * This function returns number of bytes read or written on success and negative error code on failure.
 * Check @ref LPCUSBSIO_ERR_T for more details on error code.
 */
LPCUSBSIO_API int32_t I2C_FastXfer(LPC_HANDLE hI2C, I2C_FAST_XFER_T *xfer);

/******************************************************************************
*								SPI functions
******************************************************************************/

/** @brief Initialize a SPI port.
*
* This function initializes the SPI port and the communication parameters associated
* with it.
*
* @param hUsbSio	: Handle of the LPSUSBSIO port.
* @param config	: Pointer to HID_SPI_PORTCONFIG_T structure. Members of
* HID_SPI_PORTCONFIG_T structure contains the values for SPI
* data size, clock polarity and phase
* @param portNum		: SPI port number.
*
* @returns
* This function returns a handle to SPI port object on success or NULL on failure.
* Use LPCUSBSIO_Error() function to get last error.
*/

LPCUSBSIO_API LPC_HANDLE SPI_Open(LPC_HANDLE hUsbSio, HID_SPI_PORTCONFIG_T *config, uint8_t portNum);

/** @brief Closes a SPI port.
*
* Deinitializes SPI port and frees all resources that were used by it.
*
* @param hSPI	: Handle of the SPI port.
*
* @returns
* This function returns LPCUSBSIO_OK on success and negative error code on failure.
* Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t SPI_Close(LPC_HANDLE hSPI);

/**@brief	Transmit and Receive data in SPI master mode
*
* During the transfer no code(like event handler) must change the content of the memory
* pointed to by @a xfer.The member of @a xfer, @a txBuff and @a length be
* initialized to the memory from which the SPI must pick the data to be
* transferred to slave and the number of bytes to send respectively, similarly
* @a rxBuff and @a length must have pointer to memory where data received
* from slave be stored and the number of data to get from slave respectively.
* Since SPI is full duplex transmission transmit length and receive length are the same
* and represented by the member of @a xfer, @a length.
*
* @param hSPI	: Handle of the SPI port.
* @param xfer	: Pointer to a SPI_XFER_T structure.
* @returns
* This function returns number of bytes read on success and negative error code on failure.
* Check @ref LPCUSBSIO_ERR_T for more details on error code.
*/
LPCUSBSIO_API int32_t SPI_Transfer(LPC_HANDLE hSPI, SPI_XFER_T *xfer);

/** @brief Reset SPI Controller.
*
*  @param hSPI	: A device handle returned from SPI_Open().
*
*  @returns
*  This function returns LPCUSBSIO_OK on success and negative error code on failure.
*  Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t SPI_Reset(LPC_HANDLE hSPI);

/******************************************************************************
*								GPIO functions
******************************************************************************/

/** @brief Read a GPIO port.
*
* Reads the pin status of the GPIO port mentioned by @a port. Each port has 32 pins associated with it.
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param status	: Pointer to GPIO port status, which is updated by the function.
*
* @returns
* This function returns negative error code on failure and returns the number of bytes 
* read on success Check @ref LPCUSBSIO_ERR_T for more details on error code. 
*
*/
LPCUSBSIO_API int32_t GPIO_ReadPort(LPC_HANDLE hUsbSio, uint8_t port, uint32_t* status);

/** @brief Write to a GPIO port.
*
* Write the pin status of the GPIO port mentioned by @a port. Each port has 32 pins associated with it.
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param status	: Pointer GPIO port status to be written. After writing into the GPIO port
* this value is updated with the read back from that port.
*
* @returns
* This function returns negative error code on failure and returns the number of bytes 
* written on success. Check @ref LPCUSBSIO_ERR_T for more details on error code. 
*
*/
LPCUSBSIO_API int32_t GPIO_WritePort(LPC_HANDLE hUsbSio, uint8_t port, uint32_t* status);

/** @brief Set GPIO port bits.
*
* Sets the selected pins of the GPIO port mentioned by @a port. Each port has 32 pins associated with it.
* The pins selected are indicated by the corresponding high bits of @a pins
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param pins	: Indicates which pins need to be set high by setting the corresponding bit in this variable.
*
* @returns
* This function returns negative error code on failure and returns the number of bytes updated
* on success. Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t GPIO_SetPort(LPC_HANDLE hUsbSio, uint8_t port, uint32_t pins);

/** @brief Clear GPIO port bits.
*
* Clears the selected pins of the GPIO port mentioned by @a port. Each port has 32 pins associated with it.
* The pins selected are indicated by the corresponding high bits of @a pins
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param pins	: Indicates which pins need to be cleared by setting the corresponding bit in this variable.
*
* @returns
* This function returns negative error code on failure and returns the number of bytes
* updated on success. Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t GPIO_ClearPort(LPC_HANDLE hUsbSio, uint8_t port, uint32_t pins);

/** @brief Read GPIO port direction bits.
*
* Reads the direction status for all pins of the GPIO port mentioned by @a port. Each port has 32 pins associated with it.
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param pPins	: Pointer to GPIO port direction status, which is updated by the function.
*
* @returns
* This function returns negative error code on failure and returns the number of bytes
* read on success. Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t GPIO_GetPortDir(LPC_HANDLE hUsbSio, uint8_t port, uint32_t* pPins);

/** @brief Sets GPIO port pins direction to output.
*
* Sets the direction of selected pins as output for the GPIO port mentioned by @a port. Each port has 32 pins associated with it.
* The pins selected are indicated by the corresponding high bits of @a pins
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param pins	: Indicates which pins are output pins by setting the corresponding bit in this variable.
*
* @returns
* This function returns negative error code on failure and returns the number of bytes
* updated on success. Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t GPIO_SetPortOutDir(LPC_HANDLE hUsbSio, uint8_t port, uint32_t pins);

/** @brief Sets GPIO port pins direction to input.
*
* Sets the direction of selected pins as input for the GPIO port mentioned by @a port. Each port has 32 pins associated with it.
* The pins selected are indicated by the corresponding high bits of @a pins
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param pins	: Indicates which pins are input pins by setting the corresponding bit in this variable.
*
* @returns
* This function returns negative error code on failure and returns the number of bytes
* updated on success. Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t GPIO_SetPortInDir(LPC_HANDLE hUsbSio, uint8_t port, uint32_t pins);

/** @brief Sets a specific GPIO port pin value to high.
*
* Sets a specific pin indicated by @a pin to high value for the GPIO port mentioned by @a port. 
* Each port has 32 pins associated with it.
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param pin	: The pin number which needs to be set (0 - 31).
*
* @returns
* This function returns negative error code on failure and returns the number of bytes
* updated on success. Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t GPIO_SetPin(LPC_HANDLE hUsbSio, uint8_t port, uint8_t pin);

/** @brief Reads the state of a specific GPIO port pin.
*
* Read the state of a specific pin indicated by @a pin for the GPIO port mentioned by @a port.
* Each port has 32 pins associated with it.
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param pin	: The pin number which needs to be read (0 - 31).
*
* @returns
* This function returns negative error code on failure and returns the pin state (0 or 1)
* upon on success. Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t GPIO_GetPin(LPC_HANDLE hUsbSio, uint8_t port, uint8_t pin);

/** @brief Clears a specific GPIO port pin.
*
* Clears a specific pin indicated by @a pin for the GPIO port mentioned by @a port.
* Each port has 32 pins associated with it.
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param pin	: The pin number which needs to be cleared (0 - 31).
*
* @returns
* This function returns negative error code on failure and returns the number of bytes
* updated on success. Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t GPIO_ClearPin(LPC_HANDLE hUsbSio, uint8_t port, uint8_t pin);

/** @brief Toggles the state of a specific GPIO port pin.
*
* Toggles the state of a specific pin indicated by @a pin for the GPIO port mentioned by @a port.
* Each port has 32 pins associated with it.
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param pin	: The pin number which needs to be toggled (0 - 31).
*
* @returns
* This function returns negative error code on failure and returns zero on success
* Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t GPIO_TogglePin(LPC_HANDLE hUsbSio, uint8_t port, uint8_t pin);

/** @brief Configures the IO mode for a specific GPIO port pin.
*
* Configures the IO mode of a specific pin indicated by @a pin for the GPIO port mentioned by @a port
* to the value mentioned by @a mode.
* Each port has 32 pins associated with it.
*
* @param hUsbSio: Handle to LPCUSBSIO port.
* @param port	: GPIO port number.
* @param pin	: The pin number which needs to be toggled (0 - 31).
* @param mode	: The 32 bit IO mode value that needs to updated.
*
* @returns
* This function returns negative error code on failure and returns zero on success
* Check @ref LPCUSBSIO_ERR_T for more details on error code.
*
*/
LPCUSBSIO_API int32_t GPIO_ConfigIOPin(LPC_HANDLE hUsbSio, uint8_t port, uint8_t pin, uint32_t mode);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /*__LPCUSBSIO_H*/
