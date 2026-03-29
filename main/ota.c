/*
 * Copyright (C) Damien Caliste <dcaliste@free.fr>
 *
 * You may use this file under the terms of the BSD license as follows:
 *
 * "Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Nemo Mobile nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
 */

#include "ota.h"

#include <esp_log.h>
#include <esp_app_format.h>

#define TAG "ota"
#define HASH_LEN 32 /* SHA-256 digest length */

static void print_sha256 (const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i) {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s: %s", label, hash_print);
}

esp_ota_handle_t ota_init(const esp_partition_t *partition, const char *buf, int len)
{
    esp_ota_handle_t handle = 0;
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running) {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08"PRIx32", but running from offset 0x%08"PRIx32,
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }

    if (!partition)
        return 0;
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08"PRIx32")",
             running->type, running->subtype, running->address);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%08"PRIx32,
             partition->subtype, partition->address);

    esp_app_desc_t new_app_info;
    if (len > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
        // check current version with downloading
        memcpy(&new_app_info,  buf + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t), sizeof(esp_app_desc_t));
        ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

        esp_app_desc_t running_app_info;
        if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
            ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
        }

        const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
        esp_app_desc_t invalid_app_info;
        if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
            ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
        }

        // check current version with last invalid partition
        if (last_invalid_app != NULL) {
            if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0) {
                ESP_LOGW(TAG, "New version is the same as invalid version.");
                ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                return 0;
            }
        }

        esp_err_t err = esp_ota_begin(partition, OTA_WITH_SEQUENTIAL_WRITES, &handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
            return 0;
        }
    } else {
        ESP_LOGE(TAG, "received package doesn't contains enough data");
        return 0;
    }

    return handle;
}

esp_err_t ota_finalise(esp_ota_handle_t handle, const esp_partition_t *partition)
{
    uint8_t sha_256[HASH_LEN] = { 0 };
    esp_partition_get_sha256(partition, sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");
    esp_err_t err = esp_ota_end(handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        } else {
            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Changing boot partition");
    err = esp_ota_set_boot_partition(partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t firmware_version(char buf[], size_t len)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    esp_err_t status = esp_ota_get_partition_description(running, &running_app_info);
    if (status == ESP_OK) {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
        strncpy(buf, running_app_info.version, len);
    }
    return status;
}
