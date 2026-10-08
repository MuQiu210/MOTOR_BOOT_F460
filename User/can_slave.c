/**
 *******************************************************************************
 * @file  can_slave.c
 * @brief Loader DaMiao-style CAN slave and CAN-OTA protocol implementation.
 *
 * Ports the Modbus "write register" OTA flow onto a CAN bus using the DaMiao
 * (DM) register-access frame format documented in DM-J4310P-2EC V1.0:
 *   - Read request : ID=0x7FF STD, D[0..1]=CANID_L/H, D[2]=0x33, D[3]=RID
 *   - Read response: ID=MST_ID  STD, D[0..1]=CANID_L/H, D[2]=0x33, D[3]=RID,
 *                    D[4..7]=32-bit little-endian data
 *   - Write request: ID=0x7FF STD, D[0..1]=CANID_L/H, D[2]=0x55, D[3]=RID,
 *                    D[4..7]=32-bit little-endian data
 *   - Write response: ID=MST_ID STD, D[0..1]=CANID_L/H, D[2]=0x55, D[3]=RID
 *   - Save-to-flash: ID=0x7FF STD, D[0..1]=CANID_L/H, D[2]=0xAA, D[3]=0x01
 *                    (no-op for OTA: metadata is persisted by ota_jump_app())
 *
 * The register RID table (not the DM motor register semantics) maps the OTA
 * control / size / block / MD5 registers previously carried by Modbus.
 *******************************************************************************
 */

#include "can_slave.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "bsp.h"
#include "ota.h"
#include "can.h"
#include "damiao.h"
#include "flash.h"
#include "hc32f460_flash_layout.h"
#include "nvstore.h"

/* DM frame opcodes (manual L435..467). */
#define CAN_OTA_OP_READ  (0x33U)
#define CAN_OTA_OP_WRITE (0x55U)
#define CAN_OTA_OP_SAVE  (0xAAU)

/* OTA register RID table — single 32-bit word per DM write frame. */
#define CAN_OTA_RID_CONTROL      (0x00U)
#define CAN_OTA_RID_FW_SIZE      (0x01U)
#define CAN_OTA_RID_BLOCK_OFFSET (0x02U)
#define CAN_OTA_RID_SUB_OFFSET   (0x03U)   /* legacy 4B/word path; kept for compat */
#define CAN_OTA_RID_BLOCK_DATA   (0x04U)   /* legacy 4B/word path; kept for compat */
#define CAN_OTA_RID_MD5_0        (0x05U)
#define CAN_OTA_RID_MD5_1        (0x06U)
#define CAN_OTA_RID_MD5_2        (0x07U)
#define CAN_OTA_RID_MD5_3        (0x08U)
#define CAN_OTA_RID_STATUS       (0x09U)
#define CAN_OTA_RID_MODEL_BASE   (0x0AU)
#define CAN_OTA_RID_VERSION_BASE (0x30U)
/* Fast stream opcodes (DM 0x55 write frame, RID encodes the word sub-offset).
 * Each BLOCK_STREAM frame carries 4 bytes (D[4..7]) at slot (RID-0x10)*4 into
 * s_data_buf. The host sends one block at a time without per-frame ACK, reads
 * STATUS to get the received-slot bitmap, repairs missing slots, then sends
 * CONTROL=WRITE to commit. */
#define CAN_OTA_RID_BLOCK_STREAM (0x10U)

/* OTA control command values (mirrors the Modbus OTA commands). */
#define OTA_CMD_ENTER_UPGRADE_MODE  (1U)
#define OTA_CMD_SET_FIRMWARE_SIZE   (2U)
#define OTA_CMD_ERASE_FIRMWARE      (3U)
#define OTA_CMD_WRITE_FIRMWARE_DATA (4U)
#define OTA_CMD_UPDATE_MD5          (5U)
#define OTA_CMD_SAVE_AND_JUMP_APP   (6U)
#define OTA_CMD_SET_COMPATIBILITY_ID (8U)
#define OTA_CMD_APP_RUNNING_FLAG    (7U)

/* Reported by the APP's CONTROL read while it is running. The bootloader reads
 * CONTROL back as OTA_CMD_ENTER_UPGRADE_MODE during upgrade, and as
 * OTA_CMD_APP_RUNNING_FLAG right after a successful SAVE_AND_JUMP_APP (so the
 * host can confirm the app came back up). Mirrors modbus_ota.py APP_RUNNING_FLAG. */

#define CAN_OTA_BLOCK_BYTES (128U)
#define CAN_OTA_BLOCK_WORDS (CAN_OTA_BLOCK_BYTES / 4U)   /* 32 u32 sub-words (stream slots) */
#define CAN_OTA_STREAM_MASK_COMPLETE (0xFFFFFFFFUL)

/* APP stores CAN_ID and MASTER_ID at the start of its persisted save cache. */
#define LOADER_CAN_ID_LIMIT        (0x7FFU)
#define LOADER_APP_SAVE_CACHE_SIZE (0x158U)

struct loader_app_save_cache
{
    uint16_t can_id;
    uint16_t master_id;
    uint8_t reserved[LOADER_APP_SAVE_CACHE_SIZE - (2U * sizeof(uint16_t))];
};

_Static_assert(sizeof(struct loader_app_save_cache) == LOADER_APP_SAVE_CACHE_SIZE,
               "loader APP save cache layout must match APP g_save");

static struct damiao s_damiao;
static struct loader_app_save_cache s_app_save_cache;
static struct nvstore s_app_nvstore;
static uint16_t s_loader_can_id = (uint16_t)LOADER_CAN_ID;
static uint16_t s_loader_master_id = (uint16_t)LOADER_MASTER_ID;

static const nvstore_config_t s_app_nvstore_config = {
    .bank1_addr = NVSTORE_BANK0_ADDR,
    .bank2_addr = NVSTORE_BANK1_ADDR,
    /* Keep this identical to MOTOR_APP/User/save.c. */
    .bank_size = NVSTORE_BANK0_SIZE,
    .cache = &s_app_save_cache,
    .cache_size = sizeof(s_app_save_cache),
    .flash_erase = flash_erase,
    .flash_write = flash_write,
    .active_bank_read = firmware_info_save_bank_read,
    .active_bank_write = firmware_info_save_bank_write,
};

/* OTA register mirror (32-bit per RID, DM-frame width). */
static uint32_t s_reg_control;
static uint32_t s_reg_fw_size;
static uint32_t s_reg_block_offset;
static uint32_t s_reg_sub_offset;
static uint32_t s_reg_status;
static uint8_t  s_data_buf[CAN_OTA_BLOCK_BYTES];
static uint8_t  s_candidate_model[OTA_COMPATIBILITY_ID_SIZE];
/* Bitmap of received 4-byte stream slots within the current block (32 bits).
 * Bit i set means the frame for bytes [4i, 4i+4) has arrived. */
static uint32_t s_stream_mask;

static void can_slave_send(const struct damiao_frame *frame);
static void can_slave_on_reg(const struct damiao_frame *frame);
static bool can_slave_execute_ota_control_command(uint32_t command);
static void can_slave_send_response(uint8_t opcode, uint8_t rid, uint32_t value, bool with_data);
static uint32_t can_slave_reg_read(uint8_t rid, bool *valid);

/// @brief Restore the APP CAN identifiers without writing flash from BOOT.
static void can_slave_load_ids(void)
{
    s_loader_can_id = (uint16_t)LOADER_CAN_ID;
    s_loader_master_id = (uint16_t)LOADER_MASTER_ID;

    if (nvstore_init(&s_app_nvstore, &s_app_nvstore_config) != 0)
    {
        return;
    }
    if (nvstore_read(&s_app_nvstore) != 0)
    {
        return;
    }

    /* APP treats CAN_ID=0 as unconfigured; both IDs must be valid together. */
    if ((s_app_save_cache.can_id == 0U) ||
        (s_app_save_cache.can_id > LOADER_CAN_ID_LIMIT) ||
        (s_app_save_cache.master_id > LOADER_CAN_ID_LIMIT))
    {
        return;
    }

    s_loader_can_id = s_app_save_cache.can_id;
    s_loader_master_id = s_app_save_cache.master_id;
}

/// @brief Decode a 32-bit little-endian word from a CAN payload at index 4.
static uint32_t can_slave_decode_le32(const uint8_t data[4])
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/// @brief Encode a 32-bit little-endian word into a CAN payload at index 4.
static void can_slave_encode_le32(uint8_t data[4], uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/// @brief Return whether a broadcast 0x7FF register frame is addressed to this node.
static bool can_slave_is_addressed(const struct damiao_frame *frame)
{
    uint16_t can_id_lo = (uint16_t)frame->data[0];
    uint16_t can_id_hi = (uint16_t)frame->data[1];
    uint16_t target_id = (uint16_t)((can_id_hi << 8U) | can_id_lo);

    /* DM uses the low 11 bits of the encoded CAN ID to address a slave. */
    return (target_id == s_loader_can_id);
}

/// @brief Execute one OTA control command requested through the CAN register path.
/// @param command OTA control command value.
/// @return `true` if the command succeeds, otherwise `false`.
static bool can_slave_execute_ota_control_command(uint32_t command)
{
    bool command_result = false;

    switch (command)
    {
    case OTA_CMD_ENTER_UPGRADE_MODE:
        command_result = (ota_enter_check(&g_loader_ota) == 0);
        if (command_result)
        {
            (void)memset(s_candidate_model, 0, sizeof(s_candidate_model));
        }
        break;

    case OTA_CMD_SET_COMPATIBILITY_ID:
        command_result = (ota_set_compatibility_id(&g_loader_ota,
                                                   s_candidate_model) == 0);
        break;

    case OTA_CMD_SET_FIRMWARE_SIZE:
        command_result = (ota_firmware_size(&g_loader_ota, s_reg_fw_size) == 0);
        break;

    case OTA_CMD_ERASE_FIRMWARE:
        command_result = (ota_firmware_erase(&g_loader_ota) == 0);
        break;

    case OTA_CMD_WRITE_FIRMWARE_DATA:
    {
        uint32_t block_index = s_reg_block_offset;
        uint32_t write_offset;

        if (s_stream_mask != CAN_OTA_STREAM_MASK_COMPLETE)
        {
            command_result = false;
            break;
        }

        /* Block index out of the APP range is rejected before flashing. */
        if (block_index >= (APP_B_SIZE / CAN_OTA_BLOCK_BYTES))
        {
            command_result = false;
            break;
        }
        write_offset = block_index * CAN_OTA_BLOCK_BYTES;

        command_result = (ota_firmware_write(&g_loader_ota,
                                             write_offset,
                                             s_data_buf,
                                             CAN_OTA_BLOCK_BYTES) == 0);
        /* Keep a complete buffer available for a transport retry after a flash error. */
        if (command_result)
        {
            s_stream_mask = 0U;
        }
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

/// @brief Read one OTA register by RID.
/// @param rid   Register identifier.
/// @param valid Set to `true` if the RID is readable; left unchanged otherwise.
/// @return 32-bit register value (little-endian semantics inside the CAN frame).
static uint32_t can_slave_reg_read(uint8_t rid, bool *valid)
{
    const uint8_t *md5_data;
    uint32_t value = 0U;

    switch (rid)
    {
    case CAN_OTA_RID_CONTROL:
        *valid = true;
        /* Distinguish bootloader-idle from APP-running so the host can tell them
         * apart: the APP reads CONTROL=7 (APP_RUNNING_FLAG); the bootloader's
         * idle WAIT state reads 0, while a successful SAVE_AND_JUMP_APP keeps
         * its command visible during the short jump delay. Mid-upgrade it reads
         * the current command word. This lets the host's reset handoff decide
         * correctly. */
        if ((g_loader_ota.state == OTA_STATE_WAIT) &&
            (s_reg_control != OTA_CMD_SAVE_AND_JUMP_APP))
        {
            value = 0U;
        }
        else
        {
            value = s_reg_control;
        }
        break;

    case CAN_OTA_RID_FW_SIZE:
        *valid = true;
        value = s_reg_fw_size;
        break;

    case CAN_OTA_RID_BLOCK_OFFSET:
        *valid = true;
        value = s_reg_block_offset;
        break;

    case CAN_OTA_RID_SUB_OFFSET:
        *valid = true;
        value = s_reg_sub_offset;
        break;

    case CAN_OTA_RID_STATUS:
        *valid = true;
        /* Report the per-block stream bitmap while a block is being filled so
         * the host can detect lost frames and retransmit only those slots.
         * Bits [31:0] map to the 32 4-byte words of the current 128B block. */
        value = (uint32_t)s_stream_mask;
        break;

    case CAN_OTA_RID_MD5_0:
    case CAN_OTA_RID_MD5_1:
    case CAN_OTA_RID_MD5_2:
    case CAN_OTA_RID_MD5_3:
        *valid = true;
        md5_data = ota_md5(&g_loader_ota);
        if (md5_data != NULL)
        {
            uint32_t word_index = (uint32_t)(rid - CAN_OTA_RID_MD5_0);
            (void)memcpy(&value, &md5_data[word_index * 4U], sizeof(value));
        }
        break;

    case CAN_OTA_RID_MODEL_BASE:
    case (CAN_OTA_RID_MODEL_BASE + 1U):
    case (CAN_OTA_RID_MODEL_BASE + 2U):
    case (CAN_OTA_RID_MODEL_BASE + 3U):
        *valid = true;
        (void)memcpy(&value,
                     &g_loader_identity.product_id[(rid - CAN_OTA_RID_MODEL_BASE) * 4U],
                     sizeof(value));
        break;

    case CAN_OTA_RID_VERSION_BASE:
    case (CAN_OTA_RID_VERSION_BASE + 1U):
    case (CAN_OTA_RID_VERSION_BASE + 2U):
    case (CAN_OTA_RID_VERSION_BASE + 3U):
        *valid = true;
        (void)memcpy(&value,
                     &g_loader_identity.version[(rid - CAN_OTA_RID_VERSION_BASE) * 4U],
                     sizeof(value));
        break;

    default:
        /* Unmapped RID: not readable. */
        break;
    }

    return value;
}

/// @brief Write one OTA register by RID, applying the command side-effects.
/// @param rid   Register identifier.
/// @param value 32-bit value (little-endian as received in the frame).
/// @return `true` if the write (and any triggered command) succeeded.
static bool can_slave_reg_write(uint8_t rid, uint32_t value)
{
    bool write_result = true;

    switch (rid)
    {
    case CAN_OTA_RID_CONTROL:
        write_result = can_slave_execute_ota_control_command(value);
        s_reg_control = write_result ? value : 0U;
        break;

    case CAN_OTA_RID_FW_SIZE:
        s_reg_fw_size = value;
        break;

    case CAN_OTA_RID_BLOCK_OFFSET:
        s_reg_block_offset = value;
        s_stream_mask = 0U;   /* new block selected: clear stream bitmap */
        break;

    case CAN_OTA_RID_SUB_OFFSET:
        s_reg_sub_offset = value;
        break;

    case CAN_OTA_RID_BLOCK_DATA:
    {
        uint32_t sub = s_reg_sub_offset;
        if (sub < CAN_OTA_BLOCK_WORDS)
        {
            (void)memcpy(&s_data_buf[sub * 4U], &value, sizeof(value));
        }
        else
        {
            write_result = false;
        }
        if (write_result)
        {
            s_stream_mask |= (uint32_t)(1UL << sub);
        }
        break;
    }

    case CAN_OTA_RID_MODEL_BASE:
    case (CAN_OTA_RID_MODEL_BASE + 1U):
    case (CAN_OTA_RID_MODEL_BASE + 2U):
    case (CAN_OTA_RID_MODEL_BASE + 3U):
        (void)memcpy(&s_candidate_model[(rid - CAN_OTA_RID_MODEL_BASE) * 4U],
                     &value,
                     sizeof(value));
        break;

    case CAN_OTA_RID_STATUS:
        /* STATUS is read-only from the host perspective. */
        write_result = false;
        break;

    default:
        write_result = false;
        break;
    }

    return write_result;
}

/// @brief Send one DM response frame (read or write ack) back to the master.
static void can_slave_send_response(uint8_t opcode, uint8_t rid, uint32_t value, bool with_data)
{
    struct damiao_frame frame;

    frame.id = s_loader_master_id;
    frame.ide = 0U;
    frame.rtr = 0U;
    frame.dlc = with_data ? 8U : 4U;

    frame.data[0] = (uint8_t)(s_loader_can_id & 0xFFU);
    frame.data[1] = (uint8_t)((s_loader_can_id >> 8U) & 0xFFU);
    frame.data[2] = opcode;
    frame.data[3] = rid;
    if (with_data)
    {
        can_slave_encode_le32(&frame.data[4], value);
    }
    else
    {
        frame.data[4] = 0U;
        frame.data[5] = 0U;
        frame.data[6] = 0U;
        frame.data[7] = 0U;
    }

    can_slave_send(&frame);
}

/// @brief Handle one broadcast 0x7FF register-access CAN frame addressed to this node.
static void can_slave_on_reg(const struct damiao_frame *frame)
{
    uint8_t opcode;
    uint8_t rid;
    bool ok;

    if ((frame == NULL) || (frame->dlc < 4U))
    {
        return;
    }
    if (!can_slave_is_addressed(frame))
    {
        return;
    }

    opcode = frame->data[2];
    rid = frame->data[3];

    switch (opcode)
    {
    case CAN_OTA_OP_READ:
    {
        bool valid = false;
        uint32_t value = can_slave_reg_read(rid, &valid);
        if (valid)
        {
            can_slave_send_response(CAN_OTA_OP_READ, rid, value, true);
        }
        break;
    }

    case CAN_OTA_OP_WRITE:
    {
        if (frame->dlc < 8U)
        {
            return;
        }

        /* Fast streaming path: RID >= BLOCK_STREAM carries one 4-byte word in
         * D[4..7] with the slot index encoded in the RID (0x10..0x2F -> slots
         * 0..31). The device stores the word into s_data_buf and stays silent
         * — no per-frame ACK. The host sends all 32 frames for one block,
         * reads STATUS, repairs missing slots, then sends CONTROL=WRITE_FIRMWARE_DATA.
         * This removes 32 ACK round-trips per 128B block. DM write opcode 0x55
         * and the CANID_L/H addressing header are preserved. */
        if (rid >= CAN_OTA_RID_BLOCK_STREAM)
        {
            uint32_t slot = (uint32_t)(rid - CAN_OTA_RID_BLOCK_STREAM);
            if (slot < CAN_OTA_BLOCK_WORDS)
            {
                uint32_t word = can_slave_decode_le32(&frame->data[4]);
                (void)memcpy(&s_data_buf[slot * 4U], &word, sizeof(word));
                s_stream_mask |= (uint32_t)(1UL << slot);
            }
            break;
        }

        uint32_t value = can_slave_decode_le32(&frame->data[4]);
        ok = can_slave_reg_write(rid, value);
        s_reg_status = ok ? rid : 0xFFFFFFFFU;
        can_slave_send_response(CAN_OTA_OP_WRITE, rid, 0U, false);
        break;
    }

    case CAN_OTA_OP_SAVE:
        /* Save-to-flash: metadata persistence is handled by ota_jump_app(),
         * so acknowledge without any extra flash write. */
        can_slave_send_response(CAN_OTA_OP_SAVE, 0x01U, 0U, false);
        break;

    default:
        break;
    }
}

/// @brief Send one DaMiao protocol frame through CAN1.
static void can_slave_send(const struct damiao_frame *frame)
{
    if (frame != NULL)
    {
        can_msg_t msg;

        msg.id = frame->id;
        msg.ide = frame->ide;
        msg.rtr = frame->rtr;
        msg.dlc = frame->dlc;
        (void)memcpy(msg.data, frame->data, sizeof(msg.data));
        (void)can1_send(&msg);
    }
}

/// @brief Build the current DaMiao protocol configuration for the loader OTA path.
static struct damiao_config can_slave_config(void)
{
    struct damiao_config config;

    (void)memset(&config, 0, sizeof(config));
    config.id = (uint8_t)s_loader_can_id;
    config.master_id = (uint32_t)s_loader_master_id;
    config.on_reg = can_slave_on_reg;
    config.on_send = can_slave_send;

    return config;
}

/// @brief Initialize the loader DaMiao-style CAN slave and its register mirror.
void can_slave_init(void)
{
    can_slave_load_ids();

    const struct damiao_config config = can_slave_config();

    s_reg_control = 0U;
    s_reg_fw_size = 0U;
    s_reg_block_offset = 0U;
    s_reg_sub_offset = 0U;
    s_reg_status = 0U;
    s_stream_mask = 0U;
    (void)memset(s_candidate_model, 0, sizeof(s_candidate_model));
    (void)memset(s_data_buf, 0xFFU, sizeof(s_data_buf));

    (void)damiao_init(&s_damiao, &config);
}

/// @brief Receive pending CAN frames and feed them to the DaMiao protocol parser.
void can_slave_recv(void)
{
    can_msg_t msg;
    struct damiao_frame frame;

    while (can1_recv(&msg))
    {
        frame.id = msg.id;
        frame.ide = msg.ide;
        frame.rtr = msg.rtr;
        frame.dlc = msg.dlc;
        (void)memcpy(frame.data, msg.data, sizeof(frame.data));
        (void)damiao_on_recv(&s_damiao, &frame);
    }
}

/// @brief Advance the loader CAN slave state machine by one tick.
void can_slave_step(void)
{
    /* CAN-OTA is event-driven from can_slave_recv(); no periodic work here. */
}
