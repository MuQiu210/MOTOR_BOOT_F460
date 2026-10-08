/**
 *******************************************************************************
 * @file  can_slave.h
 * @brief Loader DaMiao-style CAN slave interface for CAN-OTA.
 *******************************************************************************
 */

#ifndef __LOADER_USER_CAN_SLAVE_H__
#define __LOADER_USER_CAN_SLAVE_H__

#include <stdint.h>

/// @brief Loader CAN node identifier (11-bit, low byte only used by DM frame).
/// @note  Hard-coded for the first CAN-OTA revision. Persisting this into a
///        flash-backed save page is a later optimization (see CAN_OTA_DEV_PLAN §8.11).
#ifndef LOADER_CAN_ID
#define LOADER_CAN_ID (1U)
#endif

/// @brief Loader CAN master identifier used for response frames.
#ifndef LOADER_MASTER_ID
#define LOADER_MASTER_ID (0U)
#endif

/// @brief Initialize the DaMiao-style CAN slave used by the loader OTA path.
void can_slave_init(void);

/// @brief Receive and forward pending CAN frames to the DaMiao protocol parser.
void can_slave_recv(void);

/// @brief Advance the loader CAN slave state machine by one tick.
/// @note  Kept for API symmetry with modbus_slave_step(); currently a no-op
///        because the CAN path is event-driven inside can_slave_recv().
void can_slave_step(void);

#endif /* __LOADER_USER_CAN_SLAVE_H__ */
