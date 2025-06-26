/*
 * serial.h
 *
 *  Created on: Dec 3, 2020
 *  Author: Andrey Mihadyuk
 *
 */

#ifndef ARCH_INCLUDES_RHSP_SERIAL_H_
#define ARCH_INCLUDES_RHSP_SERIAL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef _WIN32

#include <windows.h>
#include <tchar.h>

#endif

#define RHSP_SERIAL_INFINITE_TIMEOUT -1

typedef enum {
    RHSP_SERIAL_PARITY_NONE = 0,
    RHSP_SERIAL_PARITY_ODD,
    RHSP_SERIAL_PARITY_EVEN
} RhspSerialParity;

typedef enum {
    RHSP_SERIAL_FLOW_CONTROL_NONE = 0, /* no flow control */
    RHSP_SERIAL_FLOW_CONTROL_HARDWARE, /* hardware flow control (RTS/CTS)  */
    RHSP_SERIAL_FLOW_CONTROL_SOFTWARE, /* software flow control (XON/XOFF) */
} RhspSerialFlowControl;

#ifdef __cplusplus
extern "C" {
#endif

int rhsp_serialOpenDirect(const char* serialPort, uint32_t baudrate,
                          uint32_t databits, RhspSerialParity parity,
                          uint32_t stopbits, RhspSerialFlowControl flowControl);

#ifdef __cplusplus
}
#endif

#endif /* ARCH_INCLUDES_RHSP_SERIAL_H_ */
