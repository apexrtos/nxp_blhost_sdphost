/*
 * Copyright 2020 - 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "blfwk/i2c.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static struct i2c_rdwr_ioctl_data i2c_data = { 0 };

/*******************************************************************************
 * Codes
 ******************************************************************************/

// See i2c.h for documentation of this method.
static unsigned long i2c_check_functionality(int fd)
{
    unsigned long funcs = 0;

    if (fd < 0)
    {
        return 0;
    }

    if (ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        return 0;
    }

    return funcs & I2C_FUNC_I2C;
}

// See i2c.h for documentation of this method.
int i2c_setup(int fd, uint32_t speed, uint8_t address)
{
    if (fd < 0)
    {
        return -1;
    }

    i2c_data.nmsgs = 1;
    i2c_data.msgs = (struct i2c_msg *)malloc(i2c_data.nmsgs * sizeof(struct i2c_msg));
    if (i2c_data.msgs == NULL)
    {
        return -1;
    }
    i2c_data.msgs[0].addr = (uint16_t)address;

    /*
     * I2C speed setting is not supported.
     */

    return 0;
}

// See i2c.h for documentation of this method.
int i2c_set_timeout(int fd, uint32_t milliseconds)
{
    if (fd < 0)
    {
        return -1;
    }

    /* For historical reasons, user-space sets the timeout
     * value in units of 10 ms.
     */
    return ioctl(fd, I2C_TIMEOUT, (unsigned long)milliseconds / 10);
}

// See i2c.h for documentation of this method.
int i2c_write(int fd, char *buf, int size)
{
    int ret = -1;

    if ((fd < 0) || (i2c_data.msgs == NULL))
    {
        return 0;
    }

    i2c_data.msgs[0].flags = 0; // write command
    i2c_data.msgs[0].buf = (unsigned char *)buf;
    i2c_data.msgs[0].len = size;
    ret = ioctl(fd, I2C_RDWR, (unsigned long)&i2c_data);
    if (ret < 0)
    {
        return 0;
    }

    return size;
}

// See i2c.h for documentation of this method.
int i2c_read(int fd, char *buf, int size)
{
    int ret = -1;

    if ((fd < 0) || (i2c_data.msgs == NULL))
    {
        return 0;
    }

    i2c_data.msgs[0].flags = I2C_M_RD; // read command
    i2c_data.msgs[0].buf = (unsigned char *)buf;
    i2c_data.msgs[0].len = size;
    ret = ioctl(fd, I2C_RDWR, (unsigned long)&i2c_data);
    if (ret < 0)
    {
        return 0;
    }

    return size;
}

// See i2c.h for documentation of this method.
int i2c_open(char *port)
{
    int fd;

    fd = open(port, O_RDWR);
    if (fd < 0)
    {
        fprintf(stderr, "Failed to open I2C port(%s).\n", port);
        return fd;
    }

    if (!i2c_check_functionality(fd))
    {
        fprintf(stderr, "Not an I2C port(%s).\n", port);
        return -1;
    }

    return fd;
}

// See i2c.h for documentation of this method.
int i2c_close(int fd)
{
    int ret;
    if (i2c_data.msgs != NULL)
    {
        free(i2c_data.msgs);
        i2c_data.msgs = NULL;
    }

    ret = close(fd);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to close I2C port.\n");
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
