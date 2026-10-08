/**
 *******************************************************************************
 * @file  flash.h
 * @brief Loader flash erase/program interface.
 *******************************************************************************
 */

#ifndef __LOADER_HARDWARE_FLASH_H__
#define __LOADER_HARDWARE_FLASH_H__

#include <stdint.h>

/// @brief Initialize the loader flash driver.
void flash_init(void);

/// @brief Erase a range of flash sectors.
/// @param u32_addr Erase start address.
/// @param u32_len Erase length in bytes.
/// @return Driver status code.
int32_t flash_erase(uint32_t u32_addr, uint32_t u32_len);

/// @brief Program bytes into flash.
/// @param u32_addr Write start address.
/// @param pu8_data Source buffer pointer.
/// @param u32_len Number of bytes to write.
/// @return Driver status code.
int32_t flash_write(uint32_t u32_addr, const uint8_t *pu8_data, uint32_t u32_len);

/// @brief Read bytes from flash.
/// @param u32_addr Read start address.
/// @param pu8_data Destination buffer pointer.
/// @param u32_len Number of bytes to read.
/// @return Driver status code.
int32_t flash_read(uint32_t u32_addr, uint8_t *pu8_data, uint32_t u32_len);

#endif /* __LOADER_HARDWARE_FLASH_H__ */
