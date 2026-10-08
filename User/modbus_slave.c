/**
 *******************************************************************************
 * @file  modbus_slave.c
 * @brief Loader Modbus RTU slave and OTA protocol implementation.
 *******************************************************************************
 */

#include "modbus_slave.h"

#include <errno.h>
#include <string.h>

#include "main.h"
#include "bsp.h"
#include "ota.h"
#include "modbus.h"
#include "uart3.h"
#include "hc32f460_flash_layout.h"

#define MODBUS_SLAVE_ID (1U)
#define MODBUS_SLAVE_FRAME_GAP_TICK_MS (2U)
#define MODBUS_SLAVE_STEP_TICK_MS (1U)
#define MODBUS_SLAVE_RX_BUFFER_SIZE (64U)

#define OTA_REG_CONTROL (0x0000U)
#define OTA_REG_VERSION_3 (0x0001U)
#define OTA_REG_VERSION_2 (0x0002U)
#define OTA_REG_VERSION_1 (0x0003U)
#define OTA_REG_VERSION_0 (0x0004U)
#define OTA_REG_MD5_START (0x0005U)
#define OTA_REG_MD5_END (0x000CU)
#define OTA_REG_APP_SIZE_H (0x000DU)
#define OTA_REG_APP_SIZE_L (0x000EU)
#define OTA_REG_WRITE_OFFSET (0x000FU)
#define OTA_REG_DATA_START (0x0010U)
#define OTA_REG_DATA_END (0x004FU)
#define OTA_REG_WRITE_CONTROL (0x0050U)
#define OTA_REG_CANDIDATE_ID_START (0x0051U)
#define OTA_REG_CANDIDATE_ID_END (0x0058U)
#define OTA_REG_MODEL_START (0x00F0U)
#define OTA_REG_MODEL_END (0x00F7U)
#define OTA_REG_VERSION_START (0x00F8U)
#define OTA_REG_VERSION_END (0x00FFU)

#define OTA_CMD_ENTER_UPGRADE_MODE (1U)
#define OTA_CMD_SET_FIRMWARE_SIZE (2U)
#define OTA_CMD_ERASE_FIRMWARE (3U)
#define OTA_CMD_WRITE_FIRMWARE_DATA (4U)
#define OTA_CMD_UPDATE_MD5 (5U)
#define OTA_CMD_SAVE_AND_JUMP_APP (6U)
#define OTA_CMD_SET_COMPATIBILITY_ID (8U)

typedef struct modbus_regs_t
{
    uint16_t control_status;
    uint16_t firmware_size_h;
    uint16_t firmware_size_l;
    uint16_t firmware_offset;
    uint16_t data_regs[64];
    uint16_t candidate_id_regs[8];
} modbus_regs_t;

static struct modbus_rtu_slave s_modbus_slave_handle;
static struct modbus_slave s_modbus_slave;
static modbus_regs_t s_modbus_regs;

/* Keep Modbus hooks at the bottom of static functions. */
static error_t modbus_read_hook(modbus_read *recv);
static error_t modbus_write_hook(modbus_write *write);
static bool modbus_execute_ota_control_command(uint16_t command);

static uint16_t modbus_identity_word(const uint8_t *bytes,
                                     uint16_t base,
                                     uint16_t addr)
{
    uint16_t index = (uint16_t)(addr - base);

    return (uint16_t)(((uint16_t)bytes[index * 2U] << 8U) |
                      (uint16_t)bytes[index * 2U + 1U]);
}

/// @brief Execute the OTA control command requested through Modbus.
/// @param command OTA control command value.
/// @return `true` if the command succeeds, otherwise `false`.
static bool modbus_execute_ota_control_command(uint16_t command)
{
    bool command_result = false;

    /* English comment. */
    switch (command)
    {
    case OTA_CMD_ENTER_UPGRADE_MODE:
        command_result = (ota_enter_check(&g_loader_ota) == 0);
        if (command_result)
        {
            (void)memset(s_modbus_regs.candidate_id_regs,
                         0,
                         sizeof(s_modbus_regs.candidate_id_regs));
        }
        break;

    case OTA_CMD_SET_COMPATIBILITY_ID:
    {
        uint8_t candidate_id[OTA_COMPATIBILITY_ID_SIZE];

        for (uint32_t loop = 0UL; loop < 8UL; loop++)
        {
            candidate_id[2UL * loop] =
                (uint8_t)(s_modbus_regs.candidate_id_regs[loop] >> 8U);
            candidate_id[2UL * loop + 1UL] =
                (uint8_t)(s_modbus_regs.candidate_id_regs[loop] & 0xFFU);
        }

        command_result = (ota_set_compatibility_id(&g_loader_ota, candidate_id) == 0);
        break;
    }

    case OTA_CMD_SET_FIRMWARE_SIZE:
        command_result = (ota_firmware_size(&g_loader_ota,
                                            ((uint32_t)s_modbus_regs.firmware_size_h << 16U) |
                                                (uint32_t)s_modbus_regs.firmware_size_l) == 0);
        break;

    case OTA_CMD_ERASE_FIRMWARE:
        command_result = (ota_firmware_erase(&g_loader_ota) == 0);
        break;

    case OTA_CMD_WRITE_FIRMWARE_DATA:
    {
        uint8_t write_data[sizeof(s_modbus_regs.data_regs)];
        uint32_t write_offset;
        uint32_t word_count = (sizeof(write_data) / 2UL);

        /* Loop through current range. */
        for (uint32_t loop = 0UL; loop < word_count; loop++)
        {
            write_data[2UL * loop] = (uint8_t)(s_modbus_regs.data_regs[loop] >> 8U);
            write_data[2UL * loop + 1UL] = (uint8_t)(s_modbus_regs.data_regs[loop] & 0xFFU);
        }

        write_offset = (uint32_t)s_modbus_regs.firmware_offset * (uint32_t)sizeof(write_data);
        if (write_offset >= APP_B_SIZE)
        {
            command_result = false;
            break;
        }

        command_result = (ota_firmware_write(&g_loader_ota,
                                             write_offset,
                                             write_data,
                                             sizeof(write_data)) == 0);
        break;
    }

    case OTA_CMD_UPDATE_MD5:
        command_result = (ota_md5_update(&g_loader_ota) == 0);
        break;

    case OTA_CMD_SAVE_AND_JUMP_APP:
        command_result = (ota_jump_app(&g_loader_ota, 100UL) == 0);
        break;

    default:
        command_result = false;
        break;
    }

    return command_result;
}


/// @brief Handle a Modbus register read request from the loader node.
/// @param recv Read-request descriptor.
/// @return Modbus error code for the request.
static error_t modbus_read_hook(modbus_read *recv)
{
    uint16_t addr;
    uint64_t version_value;

    if (recv == NULL)
    {
        return EINVAL;
    }

    addr = recv->addr;
    /* English comment. */
    switch (addr)
    {
    case OTA_REG_CONTROL:
        /* Read command execution status. */
        recv->data = s_modbus_regs.control_status;
        break;

    case OTA_REG_VERSION_3:
    case OTA_REG_VERSION_2:
    case OTA_REG_VERSION_1:
    case OTA_REG_VERSION_0:
        version_value = g_loader_identity.build_time;
        recv->data = (uint16_t)((version_value >> ((OTA_REG_VERSION_3 - addr) * 16U)) & 0xFFFFU);
        break;

    case OTA_REG_APP_SIZE_H:
        /* Read firmware size high 16 bits. */
        recv->data = s_modbus_regs.firmware_size_h;
        break;

    case OTA_REG_APP_SIZE_L:
        /* Read firmware size low 16 bits. */
        recv->data = s_modbus_regs.firmware_size_l;
        break;

    case OTA_REG_WRITE_OFFSET:
        /* Read firmware offset (block index). */
        recv->data = s_modbus_regs.firmware_offset;
        break;

    case OTA_REG_WRITE_CONTROL:
        recv->data = s_modbus_regs.control_status;
        break;

    default:
        if ((addr >= OTA_REG_MODEL_START) && (addr <= OTA_REG_MODEL_END))
        {
            recv->data = modbus_identity_word(g_loader_identity.product_id,
                                              OTA_REG_MODEL_START,
                                              addr);
        }
        else if ((addr >= OTA_REG_VERSION_START) && (addr <= OTA_REG_VERSION_END))
        {
            recv->data = modbus_identity_word(g_loader_identity.version,
                                              OTA_REG_VERSION_START,
                                              addr);
        }
        else if ((addr >= OTA_REG_MD5_START) && (addr <= OTA_REG_MD5_END))
        {
            const uint8_t *md5_data = ota_md5(&g_loader_ota);
            uint16_t md5_index = (uint16_t)(addr - OTA_REG_MD5_START);

            /* Read one 16-bit MD5 word. */
            if (md5_data != NULL)
            {
                recv->data = (uint16_t)(((uint16_t)md5_data[md5_index * 2U] << 8U) |
                                        (uint16_t)md5_data[md5_index * 2U + 1U]);
            }
            else
            {
                recv->data = 0U;
            }
        }
        /* Check condition and branch. */
        else if ((addr >= OTA_REG_DATA_START) && (addr <= OTA_REG_DATA_END))
        {
            /* Read one data buffer register. */
            recv->data = s_modbus_regs.data_regs[addr - OTA_REG_DATA_START];
        }
        else
        {
            /* Unmapped address returns zero. */
            recv->data = 0U;
        }
        break;
    }

    return 0;
}

/// @brief Handle a Modbus register write request from the loader node.
/// @param write Write-request descriptor.
/// @return Modbus error code for the request.
static error_t modbus_write_hook(modbus_write *write)
{
    if (write == NULL)
    {
        return EINVAL;
    }

    /* English comment. */
    switch (write->addr)
    {
    case OTA_REG_CONTROL:
    {
        bool command_result;

        /* English comment. */
        command_result = modbus_execute_ota_control_command(write->data);

        /* Check condition and branch. */
        if (command_result)
        {
            s_modbus_regs.control_status = write->data;
        }
        else
        {
            s_modbus_regs.control_status = 0U;
        }
        break;
    }

    case OTA_REG_WRITE_CONTROL:
    {
        bool command_result = false;

        if (write->data == OTA_CMD_WRITE_FIRMWARE_DATA)
        {
            command_result = modbus_execute_ota_control_command(OTA_CMD_WRITE_FIRMWARE_DATA);
        }

        s_modbus_regs.control_status = command_result ? OTA_CMD_WRITE_FIRMWARE_DATA : 0U;
        break;
    }

    case OTA_REG_APP_SIZE_H:
        s_modbus_regs.firmware_size_h = write->data;
        break;

    case OTA_REG_APP_SIZE_L:
        s_modbus_regs.firmware_size_l = write->data;
        break;

    case OTA_REG_WRITE_OFFSET:
        /* English comment. */
        s_modbus_regs.firmware_offset = write->data;
        break;

    default:
        if ((write->addr >= OTA_REG_CANDIDATE_ID_START) &&
            (write->addr <= OTA_REG_CANDIDATE_ID_END))
        {
            s_modbus_regs.candidate_id_regs[write->addr - OTA_REG_CANDIDATE_ID_START] =
                write->data;
        }
        else if ((write->addr >= OTA_REG_DATA_START) && (write->addr <= OTA_REG_DATA_END))
        {
            /* English comment. */
            s_modbus_regs.data_regs[write->addr - OTA_REG_DATA_START] = write->data;
        }
        break;
    }

    return 0;
}

/// @brief Initialize the loader Modbus RTU slave and its backing register map.
void modbus_slave_init(void)
{
    const struct modbus_rtu_slave_config config = {
        .interval = MODBUS_SLAVE_FRAME_GAP_TICK_MS,
        .on_send = uart3_send,
        .on_read = NULL,
        .on_write = NULL,
    };

    (void)memset(&s_modbus_regs, 0, sizeof(s_modbus_regs));

    (void)modbus_rtu_slave_init(&s_modbus_slave_handle, &config);

    (void)modbus_slave_node_init(&s_modbus_slave,
                                 MODBUS_SLAVE_ID,
                                 modbus_read_hook,
                                 modbus_write_hook);

    (void)modbus_rtu_slave_add_node(&s_modbus_slave_handle, &s_modbus_slave);
}

/// @brief Receive pending UART data and pass complete bytes to the Modbus parser.
void modbus_slave_recv(void)
{
    uint8_t buffer[MODBUS_SLAVE_RX_BUFFER_SIZE];
    uint32_t length = uart3_recv(buffer, sizeof(buffer));

    /* Check condition and branch. */
    if (length == 0UL)
    {
        return;
    }

    (void)modbus_rtu_slave_on_recv(&s_modbus_slave_handle, buffer, length);
}

/// @brief Advance the loader Modbus RTU state machine by one tick.
void modbus_slave_step(void)
{
    modbus_rtu_slave_on_step(&s_modbus_slave_handle, MODBUS_SLAVE_STEP_TICK_MS);
}
