/**
 *******************************************************************************
 * @file  can.h
 * @brief Loader CAN peripheral initialization and message interface.
 *******************************************************************************
 */

#ifndef __LOADER_HARDWARE_CAN_H__
#define __LOADER_HARDWARE_CAN_H__

#include <stdbool.h>
#include <stdint.h>

/// @brief Simplified CAN message descriptor (classic CAN, ≤8 bytes).
typedef struct {
    uint32_t id;        /*!< CAN identifier (11-bit if ide=0, 29-bit if ide=1) */
    uint8_t  ide;       /*!< 0 = standard ID, 1 = extended ID */
    uint8_t  rtr;       /*!< 0 = data frame, 1 = remote frame */
    uint8_t  dlc;       /*!< Data length code: 0..8 */
    uint8_t  data[8];   /*!< Payload bytes */
} can_msg_t;

/// @brief Initialize CAN1 for the loader communication path.
void can1_init(void);

/// @brief  Queue a classic CAN frame for transmission.
/// @param  msg  Frame to send (dlc ≤ 8).
/// @retval true   Frame accepted into TX FIFO.
/// @retval false  TX FIFO full; frame not sent.
bool can1_send(const can_msg_t *msg);

/// @brief  Read one frame from RX FIFO0.
/// @param  msg  Output buffer.
/// @retval true   Frame written to *msg.
/// @retval false  RX FIFO0 empty; *msg unchanged.
bool can1_recv(can_msg_t *msg);

#endif /* __LOADER_HARDWARE_CAN_H__ */
