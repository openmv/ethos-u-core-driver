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

#ifndef ETHOSU_DRIVER_COP_H
#define ETHOSU_DRIVER_COP_H

#include "ethosu_driver.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Prototypes
 ******************************************************************************/

/**
 * Verify that the custom operator payload size is valid.
 *
 * @param custom_data_size  Payload size in bytes
 * @return true if the size is at least 4 bytes and a multiple of 4, otherwise false
 */
bool ethosu_verify_cop_data_size(const int custom_data_size);

/**
 * Read the optimizer config word from a COP1 payload or a COP2 payload with a single command stream entry, without
 * executing it.
 *
 * @param custom_data_ptr   Pointer to custom operator payload data
 * @param custom_data_size  Payload size in bytes. Must be validated with ethosu_verify_cop_data_size() first
 * @param[out] cfg_out      Pointer receiving the optimizer config word on success. Unchanged on error
 * @return 0 on success, -1 if cfg_out is NULL, parsing fails, no optimizer config is found or a COP2 payload has
 *         more than one command stream entry
 */
int ethosu_get_cop_optimizer_config(const void *custom_data_ptr, size_t custom_data_size, uint32_t *cfg_out);

/**
 * Parse a COP1 or COP2 custom operator payload, verify its configuration and start its command stream on the NPU.
 *
 * @param drv               Pointer to initialized driver handle with job data prepared for execution
 * @param custom_data_ptr   Pointer to custom operator payload data
 * @param custom_data_size  Payload size in bytes. Must be validated with ethosu_verify_cop_data_size() first
 * @return 0 on success, else negative error code on parsing, configuration verification or command stream start failure
 */
int ethosu_handle_cop_payload(struct ethosu_driver *drv, const void *custom_data_ptr, size_t custom_data_size);

#ifdef __cplusplus
}
#endif

#endif // ETHOSU_DRIVER_COP_H
