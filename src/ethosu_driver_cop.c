/*
 * SPDX-FileCopyrightText: Copyright 2019-2026 Arm Limited and/or its affiliates <open-source-office@arm.com>
 * SPDX-FileCopyrightText: Copyright 2026 Youngsik Yang <vacu9708@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/******************************************************************************
 * Includes
 ******************************************************************************/

#include "ethosu_driver_cop.h"

#include "ethosu_device.h"
#include "ethosu_log.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/******************************************************************************
 * Defines
 ******************************************************************************/

#define BYTES_IN_32_BITS 4
#define MASK_16_BYTE_ALIGN (0xF)
#define OPTIMIZER_CONFIG_LENGTH_32_BIT_WORD 2
#define DRIVER_ACTION_LENGTH_32_BIT_WORD 1
#define ETHOSU_COP1_FOURCC ('1' << 24 | 'P' << 16 | 'O' << 8 | 'C') // "Custom Operator Payload 1"
#define ETHOSU_COP2_FOURCC ('2' << 24 | 'P' << 16 | 'O' << 8 | 'C') // "Custom Operator Payload 2"
#define ETHOSU_VEUM_FOURCC ('M' << 24 | 'U' << 16 | 'E' << 8 | 'V') // "Vela Ethos-U Metadata"
#define ETHOSU_COP2_SUPPORTED_MAJOR 1
#define ETHOSU_COP2_SUPPORTED_MINOR 0
#define ETHOSU_VEUM_SUPPORTED_MAJOR 1
#define ETHOSU_VEUM_SUPPORTED_MINOR 0
#define ETHOSU_COP2_FEATURE_NETWORK_WIDE_ADDRESSING 0x1
#define ETHOSU_COP2_FEATURE_LOCAL_ADDRESSING 0x2
#define ETHOSU_COP2_MAX_COMMAND_STREAM_SIZE 0x1000000 // QSIZE register range is 0 to 16 MB
#define ETHOSU_COP2_LOCAL_ADDRESSING_MIN_BASE_ADDR 5  // regions 0-2, input region 3, output region 4

/******************************************************************************
 * Types
 ******************************************************************************/

enum COP_FORMAT_e
{
    COP_FORMAT_1,
    COP_FORMAT_2,
};

// Driver actions
enum DRIVER_ACTION_e
{
    RESERVED         = 0,
    OPTIMIZER_CONFIG = 1,
    COMMAND_STREAM   = 2,
    NOP              = 5,
};

// Custom operator payload data struct
struct cop_data_s
{
    union
    {
        // Driver action data
        struct
        {
            uint8_t driver_action_command; // (valid values in DRIVER_ACTION_e)
            uint8_t reserved;

            // Driver action data
            union
            {
                // DA_CMD_OPT_CFG
                struct
                {
                    uint16_t rel_nbr : 4;
                    uint16_t patch_nbr : 4;
                    uint16_t opt_cfg_reserved : 8;
                };

                // DA_CMD_CMSTRM
                struct
                {
                    uint16_t length;
                };

                uint16_t driver_action_data;
            };
        };

        uint32_t word;
    };
};

// optimizer config struct
struct opt_cfg_s
{
    struct cop_data_s da_data;
    uint32_t cfg;
    uint32_t id;
};

enum COP2_ENTRY_TYPE_e
{
    COP2_ENTRY_RESERVED       = 0,
    COP2_ENTRY_COMMAND_STREAM = 1,
};

/*
 * COP2 payload layout.
 *
 * COP2 payload
 * |-- COP2 header                         {cop2_header_s}
 * `-- entries                             {cop2_entries_s}
 *     `-- entry                           {cop2_entry_s}
 *         |-- entry header                {cop2_entry_header_s}
 *         `-- entry data                  {cop2_command_stream_entry_s} for a COMMAND_STREAM entry
 *             |-- command stream header   {cop2_command_stream_header_s}
 *             |-- metadata
 *             |   |-- VEUM header         {veum_header_s}
 *             |   |-- VEUM data           {veum_data_s}
 *             |   `-- padding             aligns the command stream to 16 bytes
 *             `-- command stream
 */
struct cop2_header_s
{
    uint32_t fourcc;
    uint32_t version;
    uint64_t length;
};

struct cop2_entries_s
{
    const uint8_t *start;
    const uint8_t *end;
};

struct cop2_entry_header_s
{
    uint32_t type;
    uint32_t length;
};

struct cop2_entry_s
{
    struct cop2_entry_header_s header;
    const uint8_t *data;
};

struct cop2_command_stream_header_s
{
    uint32_t metadata_length;
};

struct cop2_command_stream_entry_s
{
    struct cop2_command_stream_header_s header;
    const uint8_t *metadata;
    const uint8_t *command_stream;
    size_t command_stream_size;
};

struct veum_header_s
{
    uint32_t fourcc;
    uint32_t version;
    uint32_t length;
};

struct veum_data_s
{
    uint32_t cfg;
    uint32_t id;
    uint32_t features;
    uint32_t cache_size_low;
    uint32_t cache_size_high;
};

_Static_assert(sizeof(struct cop2_header_s) == 16, "Unexpected COP2 header size");
_Static_assert(sizeof(struct cop2_entry_header_s) == 8, "Unexpected COP2 entry header size");
_Static_assert(sizeof(struct cop2_command_stream_header_s) == 4, "Unexpected COP2 command stream header size");
_Static_assert(sizeof(struct veum_header_s) == 12, "Unexpected VEUM header size");
_Static_assert(sizeof(struct veum_data_s) == 20, "Unexpected VEUM metadata data size");

/******************************************************************************
 * Static functions
 ******************************************************************************/

static bool ethosu_verify_cop_record_words(const struct cop_data_s *data_ptr,
                                           const struct cop_data_s *data_end,
                                           size_t record_words,
                                           const char *record_name)
{
    ptrdiff_t remaining_words = data_end - data_ptr;

    if (remaining_words < 0 || (size_t)remaining_words < record_words)
    {
        LOG_ERR("Custom Operator Payload truncated %s record. remaining_words=%td, expected_words=%zu",
                record_name,
                remaining_words,
                record_words);
        return false;
    }

    return true;
}

static int handle_optimizer_config(struct ethosu_driver *drv, const struct opt_cfg_s *opt_cfg_p)
{
    LOG_INFO("Optimizer release nbr: %u patch: %u", opt_cfg_p->da_data.rel_nbr, opt_cfg_p->da_data.patch_nbr);

    if (drv->dev.desc->ops->verify_optimizer_config(&drv->dev, opt_cfg_p->cfg, opt_cfg_p->id) != true)
    {
        return -1;
    }

    return 0;
}

static int handle_command_stream(struct ethosu_driver *drv, const uint8_t *cmd_stream, const int cms_length)
{
    // cms_length is number of 32bit words
    uint32_t cms_bytes = cms_length * 4;

    LOG_INFO("handle_command_stream: cmd_stream=%p, cms_length %d words (%" PRIu32 " bytes)",
             cmd_stream,
             cms_length,
             cms_bytes);

    if (0 != ((ptrdiff_t)cmd_stream & MASK_16_BYTE_ALIGN))
    {
        LOG_ERR("Command stream addr %p not aligned to 16 bytes", cmd_stream);
        return -1;
    }

    // Verify minimum 16 byte alignment for base address'
    for (int i = 0; i < drv->job.num_base_addr; i++)
    {
        if (0 != (drv->job.base_addr[i] & MASK_16_BYTE_ALIGN))
        {
            LOG_ERR("Base addr %d: 0x%" PRIx64 "not aligned to 16 bytes", i, drv->job.base_addr[i]);
            return -1;
        }
    }

    // TODO: Add call to flush/clean the command stream too?

    // Flush/clean the data cache
    ethosu_flush_dcache(drv->job.base_addr, drv->job.base_addr_size, drv->job.num_base_addr);

    // Request power gating disabled during inference run
    if (ethosu_request_power(drv))
    {
        LOG_ERR("Failed to request power");
        return -1;
    }

    drv->job.state = ETHOSU_JOB_RUNNING;

    // Inference begin callback
    ethosu_inference_begin(drv, drv->job.user_arg);

    // Execute the command stream
    drv->dev.desc->ops->run_command_stream(
        &drv->dev, cmd_stream, cms_bytes, drv->job.base_addr, drv->job.num_base_addr);

    return 0;
}

static uint32_t cop2_version_major(uint32_t version)
{
    return version & 0xffffU;
}

static uint32_t cop2_version_minor(uint32_t version)
{
    return version >> 16;
}

static size_t round_up_4(size_t value)
{
    return (value + (BYTES_IN_32_BITS - 1)) & ~((size_t)BYTES_IN_32_BITS - 1);
}

static int parse_cop2_payload(const uint8_t *data, size_t size, struct cop2_entries_s *entries)
{
    if (size < sizeof(struct cop2_header_s))
    {
        LOG_ERR("COP2 payload too short: %zu", size);
        return -1;
    }

    struct cop2_header_s header;
    memcpy(&header, data, sizeof(header));

    uint32_t major = cop2_version_major(header.version);
    uint32_t minor = cop2_version_minor(header.version);

    if (major != ETHOSU_COP2_SUPPORTED_MAJOR)
    {
        LOG_ERR("Unsupported COP2 version %" PRIu32 ".%" PRIu32, major, minor);
        return -1;
    }

    if (minor > ETHOSU_COP2_SUPPORTED_MINOR)
    {
        LOG_WARN("COP2 minor version %" PRIu32 " is newer than supported %d, unknown entries will be skipped",
                 minor,
                 ETHOSU_COP2_SUPPORTED_MINOR);
    }

    if (header.length < sizeof(struct cop2_entry_header_s))
    {
        LOG_ERR("COP2 payload does not contain any entries");
        return -1;
    }

    const size_t remaining_size = size - sizeof(header);
    if (header.length > remaining_size)
    {
        LOG_ERR("COP2 length %" PRIu64 " exceeds remaining size %zu", header.length, remaining_size);
        return -1;
    }

    entries->start = data + sizeof(header);
    entries->end   = entries->start + (size_t)header.length;

    return 0;
}

// Read the entry at data and set *next to where the following entry starts. Entries are 4 byte aligned, but the
// last entry may end without padding, so *next is clamped to the end of the entries.
static int read_cop2_entry(const uint8_t *data, const uint8_t *end, struct cop2_entry_s *entry, const uint8_t **next)
{
    const size_t remaining_size = (size_t)(end - data);
    if (remaining_size < sizeof(struct cop2_entry_header_s))
    {
        LOG_ERR("Truncated COP2 entry header");
        return -1;
    }

    struct cop2_entry_header_s header;
    memcpy(&header, data, sizeof(header));

    const size_t remaining_data_size = remaining_size - sizeof(header);
    if (header.length > remaining_data_size)
    {
        LOG_ERR("COP2 entry length %" PRIu32 " exceeds remaining size %zu", header.length, remaining_data_size);
        return -1;
    }

    if (header.type == COP2_ENTRY_RESERVED)
    {
        LOG_ERR("COP2 entry type 0 is reserved and must not be used");
        return -1;
    }

    entry->header = header;
    entry->data   = data + sizeof(header);

    const size_t padded_entry_total_size = round_up_4(sizeof(header) + (size_t)header.length);
    *next = data + ((padded_entry_total_size < remaining_size) ? padded_entry_total_size : remaining_size);

    return 0;
}

static int parse_veum_metadata(const uint8_t *metadata, size_t metadata_size, struct veum_data_s *data)
{
    if (metadata_size < sizeof(struct veum_header_s))
    {
        LOG_ERR("COP2 metadata too short: %zu", metadata_size);
        return -1;
    }

    struct veum_header_s header;
    memcpy(&header, metadata, sizeof(header));

    if (header.fourcc != ETHOSU_VEUM_FOURCC)
    {
        LOG_ERR("COP2 metadata FourCC 0x%08" PRIx32 " is not supported", header.fourcc);
        return -1;
    }

    uint32_t major = cop2_version_major(header.version);
    uint32_t minor = cop2_version_minor(header.version);

    if (major != ETHOSU_VEUM_SUPPORTED_MAJOR)
    {
        LOG_ERR("Unsupported COP2 metadata version %" PRIu32 ".%" PRIu32, major, minor);
        return -1;
    }

    if (minor > ETHOSU_VEUM_SUPPORTED_MINOR)
    {
        LOG_WARN("COP2 metadata minor version %" PRIu32
                 " is newer than supported %d, only supported fields will be used",
                 minor,
                 ETHOSU_VEUM_SUPPORTED_MINOR);
    }

    const size_t remaining_metadata_size = metadata_size - sizeof(header);
    if (header.length > remaining_metadata_size)
    {
        LOG_ERR("COP2 metadata data size %" PRIu32 " exceeds remaining metadata size %zu",
                header.length,
                remaining_metadata_size);
        return -1;
    }

    if (header.length < sizeof(struct veum_data_s))
    {
        LOG_ERR("COP2 metadata data size %" PRIu32 " is less than the required %zu bytes",
                header.length,
                sizeof(struct veum_data_s));
        return -1;
    }

    memcpy(data, metadata + sizeof(header), sizeof(*data));

    return 0;
}

static int verify_veum_metadata(struct ethosu_driver *drv, const struct veum_data_s *data)
{
    if (drv->dev.desc->ops->verify_optimizer_config(&drv->dev, data->cfg, data->id) != true)
    {
        return -1;
    }

    if ((data->features != ETHOSU_COP2_FEATURE_NETWORK_WIDE_ADDRESSING) &&
        (data->features != ETHOSU_COP2_FEATURE_LOCAL_ADDRESSING))
    {
        LOG_ERR("Unsupported COP2 metadata features: 0x%08" PRIx32, data->features);
        return -1;
    }

    // Local addressing places I/O in region 3/4, so the app must supply addresses for both.
    if ((data->features == ETHOSU_COP2_FEATURE_LOCAL_ADDRESSING) &&
        (drv->job.num_base_addr < ETHOSU_COP2_LOCAL_ADDRESSING_MIN_BASE_ADDR))
    {
        LOG_ERR("Network uses separate IO regions, but only %d base addresses were supplied", drv->job.num_base_addr);
        return -1;
    }

    // cache_size is the dedicated SRAM (region 2) size the network needs and must fit in the fast memory
    const uint64_t cache_size = ((uint64_t)data->cache_size_high << 32) | data->cache_size_low;
    if (cache_size > 0 && (drv->fast_memory == 0 || cache_size > drv->fast_memory_size))
    {
        LOG_ERR(
            "Fast memory area too small. fast_memory_size=%zu, cache_size=%" PRIu64, drv->fast_memory_size, cache_size);
        return -1;
    }

    return 0;
}

// Split a command stream entry into its VEUM metadata and command stream.
static int parse_cop2_command_stream_entry(const struct cop2_entry_s *entry,
                                           struct cop2_command_stream_entry_s *cmd_stream_entry)
{
    struct cop2_command_stream_header_s header;

    if (entry->header.length < sizeof(header))
    {
        LOG_ERR("COP2 command stream entry too short: %" PRIu32, entry->header.length);
        return -1;
    }

    memcpy(&header, entry->data, sizeof(header));

    const size_t remaining_size = entry->header.length - sizeof(header);
    if (header.metadata_length > remaining_size)
    {
        LOG_ERR("COP2 metadata_length %" PRIu32 " exceeds remaining entry size %zu",
                header.metadata_length,
                remaining_size);
        return -1;
    }

    cmd_stream_entry->header              = header;
    cmd_stream_entry->metadata            = entry->data + sizeof(header);
    cmd_stream_entry->command_stream      = cmd_stream_entry->metadata + header.metadata_length;
    cmd_stream_entry->command_stream_size = remaining_size - header.metadata_length;

    return 0;
}

static int verify_cop2_command_stream(const struct cop2_command_stream_entry_s *cmd_stream_entry)
{
    if (cmd_stream_entry->command_stream_size == 0)
    {
        LOG_ERR("COP2 command stream entry has no command stream");
        return -1;
    }

    if ((cmd_stream_entry->command_stream_size % BYTES_IN_32_BITS) != 0)
    {
        LOG_ERR("COP2 command stream size %zu is not a multiple of 4", cmd_stream_entry->command_stream_size);
        return -1;
    }

    if (cmd_stream_entry->command_stream_size > ETHOSU_COP2_MAX_COMMAND_STREAM_SIZE)
    {
        LOG_ERR("COP2 command stream size %zu exceeds %u bytes",
                cmd_stream_entry->command_stream_size,
                ETHOSU_COP2_MAX_COMMAND_STREAM_SIZE);
        return -1;
    }

    return 0;
}

static int handle_cop1_payload(struct ethosu_driver *drv, const void *custom_data_ptr, size_t custom_data_size)
{
    const struct cop_data_s *data_ptr = custom_data_ptr;

    // The caller must have verified that custom_data_size is a multiple of 4 bytes.
    const struct cop_data_s *data_end = data_ptr + (custom_data_size / sizeof(*data_ptr));

    // Skip the FourCC word, already verified by the caller
    data_ptr++;

    while (data_ptr < data_end)
    {
        switch (data_ptr->driver_action_command)
        {
        case OPTIMIZER_CONFIG:
        {
            const size_t record_words = DRIVER_ACTION_LENGTH_32_BIT_WORD + OPTIMIZER_CONFIG_LENGTH_32_BIT_WORD;
            struct opt_cfg_s opt_cfg  = {0};
            LOG_DEBUG("OPTIMIZER_CONFIG");

            if (!ethosu_verify_cop_record_words(data_ptr, data_end, record_words, "OPTIMIZER_CONFIG"))
            {
                return -1;
            }

            opt_cfg.da_data = *data_ptr;
            opt_cfg.cfg     = data_ptr[1].word;
            opt_cfg.id      = data_ptr[2].word;

            if (handle_optimizer_config(drv, &opt_cfg) < 0)
            {
                return -1;
            }
            data_ptr += record_words;
            break;
        }
        case COMMAND_STREAM:
        {
            size_t record_words;

            // Vela only supports putting one COMMAND_STREAM per op
            LOG_DEBUG("COMMAND_STREAM");
            int cms_length                = (data_ptr->reserved << 16) | data_ptr->length;
            const uint8_t *command_stream = (const uint8_t *)(data_ptr + 1);

            record_words = DRIVER_ACTION_LENGTH_32_BIT_WORD + (size_t)cms_length;
            if (!ethosu_verify_cop_record_words(data_ptr, data_end, record_words, "COMMAND_STREAM"))
            {
                return -1;
            }

            if (handle_command_stream(drv, command_stream, cms_length) < 0)
            {
                return -1;
            }
            data_ptr += record_words;
            break;
        }
        case NOP:
            LOG_DEBUG("NOP");
            data_ptr += DRIVER_ACTION_LENGTH_32_BIT_WORD;
            break;
        default:
            LOG_ERR("UNSUPPORTED driver_action_command: %u", data_ptr->driver_action_command);
            return -1;
        }
    }

    return 0;
}

// Read the optimizer config word from a COP1 payload without executing it
static int get_cop1_optimizer_config(const void *custom_data_ptr, size_t custom_data_size, uint32_t *cfg_out)
{
    const struct cop_data_s *data_ptr = custom_data_ptr;

    // The caller must have verified that custom_data_size is a multiple of 4 bytes.
    const struct cop_data_s *data_end = data_ptr + (custom_data_size / sizeof(*data_ptr));

    // Skip the FourCC word, already verified by the caller
    data_ptr++;

    // Parse Custom Operator Payload data
    while (data_ptr < data_end)
    {
        switch (data_ptr->driver_action_command)
        {
        case OPTIMIZER_CONFIG:
            if (!ethosu_verify_cop_record_words(data_ptr,
                                                data_end,
                                                DRIVER_ACTION_LENGTH_32_BIT_WORD + OPTIMIZER_CONFIG_LENGTH_32_BIT_WORD,
                                                "OPTIMIZER_CONFIG"))
            {
                return -1;
            }

            *cfg_out = data_ptr[1].word;
            return 0;
        case COMMAND_STREAM:
        {
            size_t record_words =
                DRIVER_ACTION_LENGTH_32_BIT_WORD + (size_t)((data_ptr->reserved << 16) | data_ptr->length);

            if (!ethosu_verify_cop_record_words(data_ptr, data_end, record_words, "COMMAND_STREAM"))
            {
                return -1;
            }
            data_ptr += record_words;
            break;
        }
        case NOP:
            data_ptr += DRIVER_ACTION_LENGTH_32_BIT_WORD;
            break;
        default:
            LOG_ERR("UNSUPPORTED driver_action_command: %u", data_ptr->driver_action_command);
            return -1;
        }
    }

    LOG_ERR("Could not find product config in COP data!");
    return -1;
}

static int handle_cop2_payload(struct ethosu_driver *drv, const void *custom_data_ptr, size_t custom_data_size)
{
    struct cop2_entries_s entries;
    struct cop2_command_stream_entry_s selected_entry;
    bool selected = false;

    if (parse_cop2_payload(custom_data_ptr, custom_data_size, &entries) < 0)
    {
        return -1;
    }

    // Select the one command stream entry for this NPU
    for (const uint8_t *data = entries.start, *next = NULL; data < entries.end; data = next)
    {
        struct cop2_entry_s entry;

        if (read_cop2_entry(data, entries.end, &entry, &next) < 0)
        {
            return -1;
        }

        switch (entry.header.type)
        {
        case COP2_ENTRY_COMMAND_STREAM:
        {
            struct cop2_command_stream_entry_s candidate;
            struct veum_data_s veum_data;

            if (parse_cop2_command_stream_entry(&entry, &candidate) < 0)
            {
                return -1;
            }

            if (parse_veum_metadata(candidate.metadata, candidate.header.metadata_length, &veum_data) < 0)
            {
                continue;
            }

            if (verify_veum_metadata(drv, &veum_data) < 0)
            {
                continue;
            }

            if (selected)
            {
                LOG_ERR("Multiple compatible COP2 command stream entries are not supported");
                return -1;
            }

            if (verify_cop2_command_stream(&candidate) < 0)
            {
                return -1;
            }

            selected_entry = candidate;
            selected       = true;
            break;
        }
        default:
            LOG_WARN("Skipping unknown COP2 entry type %" PRIu32, entry.header.type);
            break;
        }
    }

    if (!selected)
    {
        LOG_ERR("No COP2 command stream entry is compatible with this NPU");
        return -1;
    }

    return handle_command_stream(
        drv, selected_entry.command_stream, (int)(selected_entry.command_stream_size / BYTES_IN_32_BITS));
}

// Read the optimizer config word from a COP2 payload with a single command stream entry without executing it.
static int get_cop2_optimizer_config(const void *custom_data_ptr, size_t custom_data_size, uint32_t *cfg_out)
{
    struct cop2_entries_s entries;
    struct cop2_command_stream_entry_s cmd_stream_entry;
    struct veum_data_s veum_data;
    bool found = false;

    if (parse_cop2_payload(custom_data_ptr, custom_data_size, &entries) < 0)
    {
        return -1;
    }

    for (const uint8_t *data = entries.start, *next = NULL; data < entries.end; data = next)
    {
        struct cop2_entry_s entry;

        if (read_cop2_entry(data, entries.end, &entry, &next) < 0)
        {
            return -1;
        }

        if (entry.header.type != COP2_ENTRY_COMMAND_STREAM)
        {
            continue;
        }

        // TODO: Support multiple command stream entries so the payload can run on any NPU variant it was compiled for
        if (found)
        {
            LOG_ERR("Product config lookup among multiple COP2 command stream entries is not supported");
            return -1;
        }

        if (parse_cop2_command_stream_entry(&entry, &cmd_stream_entry) < 0)
        {
            return -1;
        }

        found = true;
    }

    if (!found)
    {
        LOG_ERR("COP2 payload does not contain a command stream entry");
        return -1;
    }

    if (parse_veum_metadata(cmd_stream_entry.metadata, cmd_stream_entry.header.metadata_length, &veum_data) < 0)
    {
        return -1;
    }

    *cfg_out = veum_data.cfg;
    return 0;
}

static int get_cop_format(const void *custom_data_ptr, enum COP_FORMAT_e *format)
{
    uint32_t fourcc;
    memcpy(&fourcc, custom_data_ptr, sizeof(fourcc));

    switch (fourcc)
    {
    case ETHOSU_COP1_FOURCC:
        *format = COP_FORMAT_1;
        return 0;
    case ETHOSU_COP2_FOURCC:
        *format = COP_FORMAT_2;
        return 0;
    default:
        LOG_ERR("Custom Operator Payload: 0x%08" PRIx32 " is not correct, expected 0x%08x or 0x%08x",
                fourcc,
                ETHOSU_COP1_FOURCC,
                ETHOSU_COP2_FOURCC);
        return -1;
    }
}

/******************************************************************************
 * Functions API
 ******************************************************************************/

bool ethosu_verify_cop_data_size(const int custom_data_size)
{
    // COP data size must be at least 4 bytes
    if (custom_data_size < BYTES_IN_32_BITS)
    {
        LOG_ERR("custom_data_size=%d < 4", custom_data_size);
        return false;
    }

    // Custom data size must be a multiple of 4
    if ((custom_data_size % BYTES_IN_32_BITS) != 0)
    {
        LOG_ERR("custom_data_size=0x%x not a multiple of 4", (unsigned)custom_data_size);
        return false;
    }

    return true;
}

int ethosu_get_cop_optimizer_config(const void *custom_data_ptr, size_t custom_data_size, uint32_t *cfg_out)
{
    if (cfg_out == NULL)
    {
        LOG_ERR("cfg_out is NULL");
        return -1;
    }

    enum COP_FORMAT_e format;
    if (get_cop_format(custom_data_ptr, &format) < 0)
    {
        return -1;
    }

    switch (format)
    {
    case COP_FORMAT_1:
        return get_cop1_optimizer_config(custom_data_ptr, custom_data_size, cfg_out);
    case COP_FORMAT_2:
        return get_cop2_optimizer_config(custom_data_ptr, custom_data_size, cfg_out);
    default:
        LOG_ERR("Unhandled COP format %d", (int)format);
        return -1;
    }
}

int ethosu_handle_cop_payload(struct ethosu_driver *drv, const void *custom_data_ptr, size_t custom_data_size)
{
    enum COP_FORMAT_e format;
    if (get_cop_format(custom_data_ptr, &format) < 0)
    {
        return -1;
    }

    switch (format)
    {
    case COP_FORMAT_1:
        return handle_cop1_payload(drv, custom_data_ptr, custom_data_size);
    case COP_FORMAT_2:
        return handle_cop2_payload(drv, custom_data_ptr, custom_data_size);
    default:
        LOG_ERR("Unhandled COP format %d", (int)format);
        return -1;
    }
}
