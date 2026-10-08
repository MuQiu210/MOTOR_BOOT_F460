/**
 *******************************************************************************
 * @file  modbus_slave.h
 * @brief Loader Modbus RTU slave interface.
 *******************************************************************************
 */

#ifndef __LOADER_USER_MODBUS_SLAVE_H__
#define __LOADER_USER_MODBUS_SLAVE_H__

#include <stdint.h>

/// @brief Initialize the Modbus RTU slave used by the loader.
void modbus_slave_init(void);

/// @brief Receive and forward pending UART bytes to the Modbus RTU parser.
void modbus_slave_recv(void);

/// @brief Advance the Modbus RTU slave state machine.
/// @param None
void modbus_slave_step(void);

#endif /* __LOADER_USER_MODBUS_SLAVE_H__ */
