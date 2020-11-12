/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __I2C_H__
#define __I2C_H__

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

//! @addtogroup i2c
//! @{

/*******************************************************************************
 * API
 ******************************************************************************/

#if __cplusplus
extern "C"
{
#endif

    //! @name I2C
    //@{

    //! @brief Configure the opened device port.
    //!
    //! @param fd Device port handler.
    //! @param speed Device clock frequency.
    //! @param address Slave device address.
    int i2c_setup(int fd, uint32_t speed, uint8_t address);

    //! @brief Set the transfer timeout.
    //!
    //! @param fd Device port handler.
    //! @param miliseconds Transfer timeout.
    int i2c_set_timeout(int fd, uint32_t miliseconds);

    //! @brief Write bytes.
    //!
    //! @param fd Device port handler.
    //! @param buf Pointer to buffer.
    //! @param size Number of bytes to write.
    int i2c_write(int fd, char *buf, int size);

    //! @brief Read bytes.
    //!
    //! @param fd Device port handler.
    //! @param buf Pointer to buffer.
    //! @param size Number of bytes to read.
    int i2c_read(int fd, char *buf, int size);

    //! @brief Open the device port.
    //!
    //! @param port Device port string.
    int i2c_open(char *port);

    //! @brief Close the device port.
    //!
    //! @param fd Device port handler.
    int i2c_close(int fd);

    //@}

#if __cplusplus
}
#endif

//! @}

#endif //__I2C_H__

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
