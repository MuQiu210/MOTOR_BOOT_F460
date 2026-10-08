/**
 *******************************************************************************
 * @file  uart3.h
 * @brief Loader USART initialization interface.
 *******************************************************************************
 */

#ifndef __LOADER_HARDWARE_UART3_H__
#define __LOADER_HARDWARE_UART3_H__

#include <stdint.h>

/// @brief Initialize UART3 for the loader console.
/// @param bps UART baud rate in bps.
void uart3_init(uint32_t bps);

/// @brief Send bytes through UART3 in blocking mode.
/// @param data Source buffer pointer.
/// @param len Number of bytes to send.
void uart3_send(const void *data, uint32_t len);

/// @brief Send bytes through UART3 in blocking mode.
/// @param data Source buffer pointer.
/// @param len Number of bytes to send.
/// @return Number of bytes received from UART3.
uint32_t uart3_recv(void *data, uint32_t len);

/// @brief Print a formatted string through UART3.
/// @param fmt `printf`-style format string.
void uart3_printf(const char *fmt, ...);

#endif /* __LOADER_HARDWARE_UART3_H__ */
