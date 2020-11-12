/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "spi.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DEFAULT_BITS_PER_WORD (8)

/*******************************************************************************
 * Variables
 ******************************************************************************/

static struct spi_ioc_transfer spi_data;

/*******************************************************************************
 * Codes
 ******************************************************************************/

// See spi.h for documentation of this method.
int spi_setup(int fd, uint32_t speed, uint32_t mode, uint32_t bits_per_word)
{
    int ret = -1;

    if (fd < 0)
    {
        return -1;
    }

    // Set phase, polarity chipselect active state and bit order
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret < 0)
    {
        return ret;
    }

    // Set bits per word
    spi_data.bits_per_word = (bits_per_word != 8) && (bits_per_word != 9) ? DEFAULT_BITS_PER_WORD : bits_per_word;
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, (unsigned long)&spi_data.bits_per_word);
    if (ret < 0)
    {
        return ret;
    }

    // Set Max speed and current speed
    spi_data.speed_hz = speed; // current speed
    return ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, (unsigned long)&speed);
}

// See spi.h for documentation of this method.
int spi_set_timeout(int fd, uint32_t milliseconds)
{
    /*
     * Linux SPI-DEV doesn't support to set timeout from user space.
     * The timeout is maintained within the device driver.
     * Different SPI drivers have different default timeout values.
     */

    return 0;
}

// See spi.h for documentation of this method.
int spi_write(int fd, char *buf, int size)
{
    if (fd < 0)
    {
        return -1;
    }

    /*
     * Do not convert a pointer type to __u64 directly. It will lead an issue for 32bit archtectures
     */
    spi_data.tx_buf = (intptr_t)buf;
    spi_data.rx_buf = (intptr_t)NULL;
    spi_data.len = size;
    spi_data.cs_change = 0;

    return ioctl(fd, SPI_IOC_MESSAGE(1), &spi_data);
}

// See spi.h for documentation of this method.
int spi_read(int fd, char *buf, int size)
{
    if (fd < 0)
    {
        return -1;
    }

    /*
     * Do not convert a pointer type to __u64 directly. It will lead an issue for 32bit archtectures
     */
    spi_data.tx_buf = (intptr_t)NULL;
    spi_data.rx_buf = (intptr_t)buf;
    spi_data.len = size;
    spi_data.cs_change = 0;

    return ioctl(fd, SPI_IOC_MESSAGE(1), &spi_data);
}

// See spi.h for documentation of this method.
int spi_open(char *port)
{
    int fd = -1;

    if (port == NULL)
    {
        return -1;
    }

    fd = open(port, O_RDWR);
    if (fd < 0)
    {
        fprintf(stderr, "Failed to open SPI port(%s).\n", port);
    }

    return fd;
}

// See spi.h for documentation of this method.
int spi_close(int fd)
{
    int ret;
    if (fd < 0)
    {
        return -1;
    }

    ret = close(fd);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to close SPI port.\n");
    }
    else
    {
        fd = -1;
    }

    return ret;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
