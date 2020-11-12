/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef __SPI_H__
#define __SPI_H__

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

//! @addtogroup spi
//! @{

/*******************************************************************************
 * API
 ******************************************************************************/

#if __cplusplus
extern "C"
{
#endif

    //! @name SPI
    //@{

    //! @brief Configure the opened device port.
    //!
    //! @param fd Device port handler.
    //! @param speed Device clock frequency.
    //! @param mode Polarity and phase mode.
    //! @param bits_per_word Transfer size.
    int spi_setup(int fd, uint32_t speed, uint32_t mode, uint32_t bits_per_word);

    //! @brief Set the transfer timeout.
    //!
    //! @param fd Device port handler.
    //! @param miliseconds Transfer timeout.
    int spi_set_timeout(int fd, uint32_t miliseconds);

    //! @brief Write bytes.
    //!
    //! @param fd Device port handler.
    //! @param buf Pointer to buffer.
    //! @param size Number of bytes to write.
    int spi_write(int fd, char *buf, int size);

    //! @brief Read bytes.
    //!
    //! @param fd Device port handler.
    //! @param buf Pointer to buffer.
    //! @param size Number of bytes to read.
    int spi_read(int fd, char *buf, int size);

    //! @brief Open the device port.
    //!
    //! @param port Device port string.
    int spi_open(char *port);

    //! @brief Close the device port.
    //!
    //! @param fd Device port handler.
    int spi_close(int fd);

    //@}

#if __cplusplus
}
#endif

//! @}

#endif //__SPI_H__

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
