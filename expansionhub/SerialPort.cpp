/*
 * serial.c
 *
 *  Created on: Dec 3, 2020
 *  Author: Andrey Mihadyuk
 */
#include <fcntl.h>
#include <sys/select.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <linux/serial.h>
#include <asm/ioctls.h>
#include <sys/ioctl.h>

#include <stdio.h>

#include "SerialPort.h"

int eh::OpenRhspSerialPort(const char* serialPortName) {
    struct termios termiosSettings;

    int fd = -1;

    /* Open serial port */
    if ((fd = open(serialPortName, O_RDWR | O_NOCTTY)) < 0) {
        return fd;
    }

    memset(&termiosSettings, 0, sizeof(termiosSettings));

    /* c_iflag */

    /* Ignore break characters */
    termiosSettings.c_iflag = IGNBRK;

    /* c_oflag */
    termiosSettings.c_oflag = 0;

    /* c_lflag */
    termiosSettings.c_lflag = 0;

    /* c_cflag */
    /* Enable receiver, ignore modem control lines */
    termiosSettings.c_cflag = CREAD | CLOCAL;

    /* Databits */
    termiosSettings.c_cflag |= CS8;

    /* Baudrate */
    cfsetispeed(&termiosSettings, B460800);
    cfsetospeed(&termiosSettings, B460800);

    /* Set termios attributes */
    if (tcsetattr(fd, TCSANOW, &termiosSettings) < 0) {
        close(fd);
        return -1;
    }

    struct serial_struct serialCfg;

    ioctl(fd, TIOCGSERIAL, &serialCfg);
    serialCfg.flags |= ASYNC_LOW_LATENCY;
    int ret = ioctl(fd, TIOCSSERIAL, &serialCfg);
    printf("TIOCSSERIAL %d %d\n", ret, errno);

    return fd;
}
