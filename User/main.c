/**
 *******************************************************************************
 * @file  main.c
 * @brief Main program.
 * @verbatim
 *   Change Logs:
 *   Date             Author          Notes
 *   2026-02-10       CDT             First version
 * @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"
#include "bsp.h"
#include "flash.h"
#include "firmware_storage.h"
#include "hc32f460_flash_layout.h"
#include "modbus_slave.h"
#include "can_slave.h"

#include <string.h>

#include "md5.h"

#define LOADER_OTA_STARTUP_TIMEOUT (200UL)
#define LOADER_APP_RAM_START_ADDR (0x1FFF8000UL)
#define LOADER_APP_RAM_END_ADDR (0x20027000UL)

typedef void (*loader_app_entry_t)(void);

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
ota_t g_loader_ota;

static void loader_jump_app(uint32_t app_base_addr);
static bool loader_app_identity_validate(uint32_t app_base_addr,
                                         uint32_t firmware_length);
static bool loader_image_content_validate(uint32_t app_base_addr,
                                          uint32_t firmware_length,
                                          const uint8_t expected_md5[16]);
static bool loader_slot_content_validate(const struct firmware_storage_info *info,
                                         uint32_t slot,
                                         bool allow_staged);
static bool loader_recover_app_slots(void);
static bool loader_firmware_info_write(uint32_t length,
                                       const uint8_t md5[16]);

static const uint8_t s_loader_compatibility_id[OTA_COMPATIBILITY_ID_SIZE] =
    FIRMWARE_PRODUCT_ID;

static const ota_config_t s_loader_ota_config = {
    .app_base_addr = APP_B_BASE_ADDR,
    .app_end_addr = APP_B_END_ADDR,
    .active_app_base_addr = APP_A_BASE_ADDR,
    .active_app_end_addr = APP_A_END_ADDR,
    .flash_erase = flash_erase,
    .flash_write = flash_write,
    .firmware_info_read = firmware_info_recv,
    .firmware_info_write = loader_firmware_info_write,
    .compatibility_id = s_loader_compatibility_id,
    .image_identity_validate = loader_app_identity_validate,
    .on_jump_app = loader_jump_app,
};

static bool loader_app_identity_validate(uint32_t app_base_addr,
                                         uint32_t firmware_length)
{
    const struct firmware_identity *identity;
    uint32_t app_end_addr;
    uint32_t identity_offset = APP_IDENTITY_OFFSET;

    if (app_base_addr == APP_A_BASE_ADDR)
    {
        app_end_addr = APP_A_END_ADDR;
    }
    else if (app_base_addr == APP_B_BASE_ADDR)
    {
        app_end_addr = APP_B_END_ADDR;
    }
    else
    {
        return false;
    }

    if (firmware_length < (identity_offset + FIRMWARE_IDENTITY_TOTAL_SIZE))
    {
        return false;
    }

    if ((identity_offset > (app_end_addr - app_base_addr + 1UL)) ||
        ((identity_offset + FIRMWARE_IDENTITY_TOTAL_SIZE) >
         (app_end_addr - app_base_addr + 1UL)))
    {
        return false;
    }

    identity = (const struct firmware_identity *)(uintptr_t)(app_base_addr + identity_offset);
    if (memcmp(identity->magic, FIRMWARE_IDENTITY_MAGIC, FIRMWARE_IDENTITY_MAGIC_SIZE) != 0)
    {
        return false;
    }

    if (memcmp(identity->product_id,
               s_loader_compatibility_id,
               OTA_COMPATIBILITY_ID_SIZE) != 0)
    {
        return false;
    }

    return memcmp(identity->version, "APP-", 4U) == 0;
}

static bool loader_image_content_validate(uint32_t app_base_addr,
                                          uint32_t firmware_length,
                                          const uint8_t expected_md5[16])
{
    uint32_t app_end_addr;
    uint8_t calculated_md5[16];

    if (app_base_addr == APP_A_BASE_ADDR)
    {
        app_end_addr = APP_A_END_ADDR;
    }
    else if (app_base_addr == APP_B_BASE_ADDR)
    {
        app_end_addr = APP_B_END_ADDR;
    }
    else
    {
        return false;
    }

    if ((expected_md5 == NULL) || (firmware_length == 0UL) ||
        (firmware_length > (app_end_addr - app_base_addr + 1UL)))
    {
        return false;
    }

    if (md5(calculated_md5,
            (const void *)(uintptr_t)app_base_addr,
            firmware_length) != 0)
    {
        return false;
    }

    return (memcmp(calculated_md5, expected_md5, sizeof(calculated_md5)) == 0) &&
           loader_app_identity_validate(app_base_addr, firmware_length);
}

static bool loader_slot_content_validate(const struct firmware_storage_info *info,
                                         uint32_t slot,
                                         bool allow_staged)
{
    uint32_t app_base_addr;
    uint32_t image_state;

    if ((info == NULL) || (slot > FIRMWARE_SLOT_B))
    {
        return false;
    }

    image_state = info->slot[slot].state;
    if ((image_state != FIRMWARE_SLOT_STATE_VALID) &&
        !(allow_staged && (image_state == FIRMWARE_SLOT_STATE_STAGED)))
    {
        return false;
    }

    app_base_addr = (slot == FIRMWARE_SLOT_A) ? APP_A_BASE_ADDR : APP_B_BASE_ADDR;
    return loader_image_content_validate(app_base_addr,
                                         info->slot[slot].length,
                                         info->slot[slot].md5);
}

static bool loader_recover_app_slots(void)
{
    struct firmware_storage_info info;
    bool app_a_valid;
    bool app_b_valid;
    bool copy_required;

    if (!firmware_info_load(&info))
    {
        return false;
    }

    app_a_valid = loader_slot_content_validate(&info, FIRMWARE_SLOT_A, false);
    app_b_valid = loader_slot_content_validate(&info, FIRMWARE_SLOT_B, true);
    copy_required = app_b_valid &&
                    ((info.slot[FIRMWARE_SLOT_B].state == FIRMWARE_SLOT_STATE_STAGED) ||
                     !app_a_valid);

    if (!copy_required)
    {
        return app_a_valid;
    }

    if (!firmware_backup_copy(APP_B_BASE_ADDR,
                              APP_A_BASE_ADDR,
                              info.slot[FIRMWARE_SLOT_B].length) ||
        !loader_image_content_validate(APP_A_BASE_ADDR,
                                       info.slot[FIRMWARE_SLOT_B].length,
                                       info.slot[FIRMWARE_SLOT_B].md5))
    {
        return false;
    }

    return firmware_info_commit_primary(info.slot[FIRMWARE_SLOT_B].length,
                                        info.slot[FIRMWARE_SLOT_B].md5);
}

static bool loader_firmware_info_write(uint32_t length,
                                       const uint8_t md5[16])
{
    if (!loader_image_content_validate(APP_B_BASE_ADDR, length, md5) ||
        !firmware_info_stage_backup(length, md5))
    {
        return false;
    }

    if (!firmware_backup_copy(APP_B_BASE_ADDR, APP_A_BASE_ADDR, length) ||
        !loader_image_content_validate(APP_A_BASE_ADDR, length, md5))
    {
        return false;
    }

    return firmware_info_commit_primary(length, md5);
}

static void loader_jump_app(uint32_t app_base_addr)
{
    uint32_t app_stack = *(const uint32_t *)(uintptr_t)app_base_addr;
    uint32_t app_reset;
    loader_app_entry_t app_entry;

    if (app_base_addr != APP_A_BASE_ADDR)
    {
        return;
    }

    app_reset = *(const uint32_t *)(uintptr_t)(app_base_addr + sizeof(uint32_t));

    if ((app_stack < LOADER_APP_RAM_START_ADDR) || (app_stack > LOADER_APP_RAM_END_ADDR) ||
        ((app_reset < APP_A_BASE_ADDR) || (app_reset > APP_A_END_ADDR)))
    {
        return;
    }

    __disable_irq();
    bsp_deinit();

    for (uint32_t i = 0UL; i < (sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0])); i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    SCB->VTOR = app_base_addr;
    __set_MSP(app_stack);
    __DSB();
    __ISB();

    app_entry = (loader_app_entry_t)(uintptr_t)app_reset;
    app_entry();
    while(1)
    {
    }
}

/// @brief Loader entry point.
/// @return This function does not return during normal operation.
int main(void)
{
    bsp_init();
    (void)loader_recover_app_slots();
    ota_init(&g_loader_ota, &s_loader_ota_config, LOADER_OTA_STARTUP_TIMEOUT);

    modbus_slave_init();
    can_slave_init();

    while (1)
    {
        modbus_slave_recv();
        can_slave_recv();
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0UL)
        {
            modbus_slave_step();
            can_slave_step();
            ota_on_step(&g_loader_ota);
        }
    }
}
