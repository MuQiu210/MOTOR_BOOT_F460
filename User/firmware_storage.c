/// @file  firmware_storage.c
/// @brief Redundant INFO metadata, shared parameter-bank selection and image copy.

#include "firmware_storage.h"

#include <stddef.h>
#include <string.h>

#include "flash.h"
#include "hc32f460_flash_layout.h"

#define FIRMWARE_INFO_RECORD_A_ADDR (FIRMWARE_INFO_ADDR)
#define FIRMWARE_INFO_RECORD_B_ADDR (FIRMWARE_INFO_ADDR + HC32F460_FLASH_SECTOR_SIZE)
#define FIRMWARE_BACKUP_COPY_BYTES  (256UL)

static uint8_t s_firmware_copy_buffer[FIRMWARE_BACKUP_COPY_BYTES];

static uint32_t firmware_info_crc32(const struct firmware_storage_info *info)
{
    const uint8_t *data = (const uint8_t *)info;
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint32_t index = 0UL; index < offsetof(struct firmware_storage_info, crc32); index++)
    {
        crc ^= data[index];
        for (uint32_t bit = 0UL; bit < 8UL; bit++)
        {
            crc = (crc & 1UL) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }

    return ~crc;
}

static void firmware_info_reset(struct firmware_storage_info *info)
{
    (void)memset(info, 0, sizeof(*info));
    info->magic = FIRMWARE_STORAGE_MAGIC;
    info->format = FIRMWARE_STORAGE_FORMAT;
    info->save_bank = 0xFFFFFFFFUL;
}

static bool firmware_info_record_valid(const struct firmware_storage_info *info)
{
    return (info->magic == FIRMWARE_STORAGE_MAGIC) &&
           (info->format == FIRMWARE_STORAGE_FORMAT) &&
           (info->crc32 == firmware_info_crc32(info));
}

static const struct firmware_storage_info *firmware_info_record(uint32_t addr)
{
    return (const struct firmware_storage_info *)(uintptr_t)addr;
}

bool firmware_info_load(struct firmware_storage_info *info)
{
    const struct firmware_storage_info *record_a;
    const struct firmware_storage_info *record_b;
    bool valid_a;
    bool valid_b;

    if (info == NULL)
    {
        return false;
    }

    record_a = firmware_info_record(FIRMWARE_INFO_RECORD_A_ADDR);
    record_b = firmware_info_record(FIRMWARE_INFO_RECORD_B_ADDR);
    valid_a = firmware_info_record_valid(record_a);
    valid_b = firmware_info_record_valid(record_b);

    if (!valid_a && !valid_b)
    {
        firmware_info_reset(info);
        return false;
    }

    if (valid_a && (!valid_b || ((int32_t)(record_a->sequence - record_b->sequence) > 0)))
    {
        (void)memcpy(info, record_a, sizeof(*info));
    }
    else
    {
        (void)memcpy(info, record_b, sizeof(*info));
    }

    return true;
}

bool firmware_info_store(struct firmware_storage_info *info)
{
    struct firmware_storage_info current;
    struct firmware_storage_info readback;
    bool has_current;
    uint32_t current_addr = 0UL;
    uint32_t target_addr;

    if (info == NULL)
    {
        return false;
    }

    has_current = firmware_info_load(&current);
    if (has_current)
    {
        const struct firmware_storage_info *record_a =
            firmware_info_record(FIRMWARE_INFO_RECORD_A_ADDR);
        const struct firmware_storage_info *record_b =
            firmware_info_record(FIRMWARE_INFO_RECORD_B_ADDR);

        if (firmware_info_record_valid(record_a) &&
            (!firmware_info_record_valid(record_b) ||
             ((int32_t)(record_a->sequence - record_b->sequence) > 0)))
        {
            current_addr = FIRMWARE_INFO_RECORD_A_ADDR;
        }
        else
        {
            current_addr = FIRMWARE_INFO_RECORD_B_ADDR;
        }
        target_addr = (current_addr == FIRMWARE_INFO_RECORD_A_ADDR) ?
                      FIRMWARE_INFO_RECORD_B_ADDR : FIRMWARE_INFO_RECORD_A_ADDR;
        info->sequence = current.sequence + 1UL;
    }
    else
    {
        target_addr = FIRMWARE_INFO_RECORD_A_ADDR;
        info->sequence = 1UL;
    }

    info->magic = FIRMWARE_STORAGE_MAGIC;
    info->format = FIRMWARE_STORAGE_FORMAT;
    info->crc32 = firmware_info_crc32(info);

    if ((flash_erase(target_addr, HC32F460_FLASH_SECTOR_SIZE) != 0) ||
        (flash_write(target_addr,
                     (const uint8_t *)info,
                     sizeof(*info)) != 0))
    {
        return false;
    }

    (void)memcpy(&readback,
                 (const void *)(uintptr_t)target_addr,
                 sizeof(readback));
    return firmware_info_record_valid(&readback) &&
           (readback.sequence == info->sequence);
}

void firmware_info_recv(uint32_t *length, uint8_t md5[16])
{
    struct firmware_storage_info info;
    bool valid = firmware_info_load(&info) &&
                 (info.slot[FIRMWARE_SLOT_A].state == FIRMWARE_SLOT_STATE_VALID) &&
                 (info.slot[FIRMWARE_SLOT_A].length != 0UL);

    if (length != NULL)
    {
        *length = valid ? info.slot[FIRMWARE_SLOT_A].length : 0UL;
    }
    if (md5 != NULL)
    {
        if (valid)
        {
            (void)memcpy(md5, info.slot[FIRMWARE_SLOT_A].md5, 16U);
        }
        else
        {
            (void)memset(md5, 0, 16U);
        }
    }
}

uint8_t firmware_info_save_bank_read(void)
{
    struct firmware_storage_info info;

    if (!firmware_info_load(&info) || (info.save_bank > 1UL))
    {
        return FIRMWARE_SAVE_BANK_INVALID;
    }

    return (uint8_t)info.save_bank;
}

bool firmware_info_save_bank_write(uint8_t bank_index)
{
    struct firmware_storage_info info;

    if (bank_index > 1U)
    {
        return false;
    }

    (void)firmware_info_load(&info);
    info.save_bank = bank_index;
    return firmware_info_store(&info);
}

bool firmware_info_stage_backup(uint32_t length, const uint8_t md5[16])
{
    struct firmware_storage_info info;

    if ((md5 == NULL) || (length == 0UL) || (length > APP_B_SIZE))
    {
        return false;
    }

    (void)firmware_info_load(&info);
    info.slot[FIRMWARE_SLOT_B].length = length;
    (void)memcpy(info.slot[FIRMWARE_SLOT_B].md5, md5, 16U);
    info.slot[FIRMWARE_SLOT_B].state = FIRMWARE_SLOT_STATE_STAGED;
    return firmware_info_store(&info);
}

bool firmware_info_commit_primary(uint32_t length, const uint8_t md5[16])
{
    struct firmware_storage_info info;

    if ((md5 == NULL) || (length == 0UL) || (length > APP_A_SIZE) ||
        (length > APP_B_SIZE))
    {
        return false;
    }

    (void)firmware_info_load(&info);
    info.slot[FIRMWARE_SLOT_A].length = length;
    (void)memcpy(info.slot[FIRMWARE_SLOT_A].md5, md5, 16U);
    info.slot[FIRMWARE_SLOT_A].state = FIRMWARE_SLOT_STATE_VALID;
    info.slot[FIRMWARE_SLOT_B].length = length;
    (void)memcpy(info.slot[FIRMWARE_SLOT_B].md5, md5, 16U);
    info.slot[FIRMWARE_SLOT_B].state = FIRMWARE_SLOT_STATE_VALID;
    return firmware_info_store(&info);
}

bool firmware_backup_copy(uint32_t source_addr, uint32_t target_addr, uint32_t length)
{
    uint32_t offset;
    uint32_t source_size;
    uint32_t target_size;

    if ((source_addr == target_addr) || (length == 0UL) ||
        ((source_addr != APP_A_BASE_ADDR) && (source_addr != APP_B_BASE_ADDR) ) ||
        ((target_addr != APP_A_BASE_ADDR) && (target_addr != APP_B_BASE_ADDR)))
    {
        return false;
    }

    source_size = (source_addr == APP_A_BASE_ADDR) ? APP_A_SIZE : APP_B_SIZE;
    target_size = (target_addr == APP_A_BASE_ADDR) ? APP_A_SIZE : APP_B_SIZE;
    if ((length > source_size) || (length > target_size))
    {
        return false;
    }

    if (flash_erase(target_addr, target_size) != 0)
    {
        return false;
    }

    for (offset = 0UL; offset < length; offset += FIRMWARE_BACKUP_COPY_BYTES)
    {
        uint32_t chunk_length = length - offset;
        uint32_t program_length;

        if (chunk_length > FIRMWARE_BACKUP_COPY_BYTES)
        {
            chunk_length = FIRMWARE_BACKUP_COPY_BYTES;
        }
        (void)memcpy(s_firmware_copy_buffer,
                     (const void *)(uintptr_t)(source_addr + offset),
                     chunk_length);
        program_length = (chunk_length + 3UL) & ~3UL;
        if (program_length > chunk_length)
        {
            (void)memset(&s_firmware_copy_buffer[chunk_length],
                         0xFF,
                         program_length - chunk_length);
        }
        if (flash_write(target_addr + offset,
                        s_firmware_copy_buffer,
                        program_length) != 0)
        {
            return false;
        }
    }

    return true;
}
