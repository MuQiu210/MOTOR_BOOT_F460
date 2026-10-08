/**
 *******************************************************************************
 * @file  bsp.h
 * @brief Loader board support package interface.
 *******************************************************************************
 */

#ifndef __LOADER_USER_BSP_H__
#define __LOADER_USER_BSP_H__

#include <stdbool.h>
#include <stdint.h>

#include "firmware_identity.h"
#include "firmware_storage.h"

/// @brief Initialize loader board support peripherals.
void bsp_init(void);
/// @brief Deinitialize loader board support peripherals before jumping away.
void bsp_deinit(void);
/// @brief Read the stored firmware length and MD5 metadata.
/// @param length Optional destination for firmware length.
/// @param md5 Optional destination for the 16-byte firmware MD5.
void firmware_info_recv(uint32_t *length, uint8_t md5[16]);
extern const struct firmware_identity g_loader_identity;

#endif /* __LOADER_USER_BSP_H__ */
