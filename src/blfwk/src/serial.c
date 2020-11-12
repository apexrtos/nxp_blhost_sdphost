/*
 * This file is part of the Bus Pirate project (http://code.google.com/p/the-bus-pirate/).
 *
 * Written and maintained by the Bus Pirate project and http://dangerousprototypes.com
 *
 * To the extent possible under law, the project has
 * waived all copyright and related or neighboring rights to Bus Pirate. This
 * work is published from United States.
 *
 * For details see: http://creativecommons.org/publicdomain/zero/1.0/.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */
/*
 * OS independent serial interface
 *
 * Heavily based on Pirate-Loader:
 * http://the-bus-pirate.googlecode.com/svn/trunk/bootloader-v4/pirate-loader/source/pirate-loader.c
 *
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "blfwk/serial.h"

#ifdef LINUX
#include <termios.h>
#endif

int serial_setup(int fd, speed_t speed)
{
#if defined(WIN32)
    COMMTIMEOUTS timeouts;
    DCB dcb = { 0 };
    HANDLE hCom = (HANDLE)fd;

    dcb.DCBlength = sizeof(dcb);

    dcb.BaudRate = speed;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    if (!SetCommState(hCom, &dcb))
    {
        return -1;
    }

    // These timeouts mean:
    // read: return immediately with whatever data is available, if any
    // write: timeouts not used
    // reference: http://www.robbayer.com/files/serial-win.pdf
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;

    if (!SetCommTimeouts(hCom, &timeouts))
    {
        return -1;
    }

#elif defined(LINUX) || defined(MACOSX)
    struct termios tty;

    memset(&tty, 0x00, sizeof(tty));
    cfmakeraw(&tty);

    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tty.c_cflag |= (CS8 | CLOCAL | CREAD | HUPCL);
    tty.c_oflag = 0;
    tty.c_lflag = 0;

#if defined(LINUX)
    // Non standard baud rates depend on the specific system.
    // Only supports standard baud rates.
    switch (speed)
    {
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        case 230400:
            speed = B230400;
            break;
        case 460800:
            speed = B460800;
            break;
        case 500000:
            speed = B500000;
            break;
        case 576000:
            speed = B576000;
            break;
        case 921600:
            speed = B921600;
            break;
        case 1000000:
            speed = B1000000;
            break;
        case 1152000:
            speed = B1152000;
            break;
        case 1500000:
            speed = B1500000;
            break;
        case 2000000:
            speed = B2000000;
            break;
        case 2500000:
            speed = B2500000;
            break;
        case 3000000:
            speed = B3000000;
            break;
        case 3500000:
            speed = B3500000;
            break;
        case 4000000:
            speed = B4000000;
            break;
        default:
            printf("Warning: unsupported standard baud rate(%d), set to default(57600)\n", speed);
            speed = B57600;
            break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
#elif defined(MACOSX)
    // Set a dummy speed here, the real speed will be set by IOSSIOSPEED
    cfsetospeed(&tty, B57600);
    cfsetispeed(&tty, B57600);
#endif // LINUX

    // Completely non-blocking read
    // VMIN = 0 and VTIME = 0
    // Completely non-blocking read
    // reference: http://www.unixwiz.net/techtips/termios-vmin-vtime.html
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSAFLUSH, &tty) < 0)
    {
        return -1;
    }

#if defined(MACOSX)
    if (ioctl(fd, IOSSIOSPEED, &speed) == -1)
    {
        return -1;
    }
#endif // MACOSX
#endif // WIN32

#if defined(MAXOSX)
    // Set the receive latency to 1us which is used by serial driver to determine how often to dequeue characters
    // received by the hardware.
    unsigned long us = 1UL;
    if (ioctl(fd, IOSSDATALAT, &us) < 0)
    {
        return -1;
    }
#endif // MAXOSX

    return 0;
}

int serial_set_read_timeout(int fd, uint32_t timeoutMs)
{
#if defined(WIN32)
    COMMTIMEOUTS timeouts;
    HANDLE hCom = (HANDLE)fd;

    // These timeouts mean:
    // read: return if:
    //  1. Inter-character timeout exceeds ReadIntervalTimeout
    //  2. Total timeout exceeds (ReadIntervalTimeout*ReadTotalTimeoutMultiplier*number of characters) +
    //  ReadTotalTimeoutConstant
    // In practice it seems that no matter how many characters you ask for, if no characters appear on the interface
    // then
    // only ReadTotalTimeoutConstant applies.
    // write: timeouts not used
    // reference: http://www.robbayer.com/files/serial-win.pdf
    if (timeoutMs != 0)
    {
        timeouts.ReadIntervalTimeout = 1000;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.ReadTotalTimeoutConstant = timeoutMs;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant = 0;
    }
    else
    {
        // Need a seperate case for timeoutMs == 0
        // setting all these values to 0 results in no timeout
        // so set them to a minimum value, this will return immediately
        // if there is no data available
        timeouts.ReadIntervalTimeout = 1;
        timeouts.ReadTotalTimeoutMultiplier = 1;
        timeouts.ReadTotalTimeoutConstant = 1;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant = 0;
    }

    if (!SetCommTimeouts(hCom, &timeouts))
    {
        return -1;
    }

#elif defined(LINUX) || defined(MAXOSX)
    struct termios tty;

    memset(&tty, 0x00, sizeof(tty));
    tcgetattr(fd, &tty);

    // Completely non-blocking read
    // VMIN = 0 and VTIME > 0
    // Pure timed read
    // reference: http://www.unixwiz.net/techtips/termios-vmin-vtime.html
    if (timeoutMs && (timeoutMs < 100))
    {
        // since the lowest resolution this handles is .1 seconds we will set it to that for any non zero
        // timeout value less than 100ms
        tty.c_cc[VTIME] = 1;
    }
    else
    {
        tty.c_cc[VTIME] = (timeoutMs / 100); // in 0.1 sec intervals
    }

    tty.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSAFLUSH, &tty) < 0)
    {
        return -1;
    }

#endif // WIN32

    return 0;
}

int serial_write(int fd, char *buf, int size)
{
#ifdef WIN32
    HANDLE hCom = (HANDLE)fd;
    unsigned long bwritten = 0;

    if (!WriteFile(hCom, buf, size, &bwritten, NULL))
    {
        return 0;
    }
    else
    {
        return bwritten;
    }
#else
    return write(fd, buf, size);
#endif
}

int serial_read(int fd, char *buf, int size)
{
#ifdef WIN32
    HANDLE hCom = (HANDLE)fd;
    unsigned long bread = 0;

    if (!ReadFile(hCom, buf, size, &bread, NULL))
    {
        return 0;
    }
    else
    {
        return bread;
    }
#else
    int len = 0;
    int ret = 0;
    int timeout = 0;

    while (len < size)
    {
        ret = read(fd, buf + len, size - len);
        if (ret == -1)
        {
            return -1;
        }

        if (ret == 0)
        {
            timeout++;

#if defined(MACOSX)
            // There is a huge gap every 32 bytes on some MACOS. So use an extremely large value to workaround it.
            if (timeout >= 1000 * 1000)
            {
                break;
            }
#else
            if (timeout >= 10)
            {
                break;
            }
#endif

            continue;
        }
        else
        {
            // Reset the timeout, once data byte(s) is(are) received.
            timeout = 0;
        }

        len += ret;
    }

    return len;
#endif
}

int serial_open(char *port)
{
    int fd;
#if defined(WIN32)
    static char full_path[32] = { 0 };

    HANDLE hCom = NULL;

    if (port[0] != '\\')
    {
        _snprintf(full_path, sizeof(full_path) - 1, "\\\\.\\%s", port);
        port = full_path;
    }

#pragma warning(suppress : 6053)
    hCom = CreateFileA(port, GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (!hCom || hCom == INVALID_HANDLE_VALUE)
    {
        fd = -1;
    }
    else
    {
        fd = (int)hCom;
    }
#elif defined(LINUX)
    fd = open(port, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        fprintf(stderr, "Could not open serial port.\n");
        return -1;
    }
#elif defined(MACOSX)
    // There is an issue in some MACOS about the blocking operations for serial devices. So use non-blocking to avoid
    // it.
    fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1)
    {
        fprintf(stderr, "Could not open serial port.\n");
        return -1;
    }

    // O_NONBLOCK is required for the OPEN operation. Clear the O_NONBLOCK flag, so subsequent operation will block.
    if (fcntl(fd, F_SETFL, 0) == -1)
    {
        fprintf(stderr, "Failed to set non-blocking mode to the serial port.\n");
        return -1;
    }
#endif
    return fd;
}

int serial_close(int fd)
{
#ifdef WIN32
    HANDLE hCom = (HANDLE)fd;

    CloseHandle(hCom);
#else
    close(fd);
#endif
    return 0;
}
