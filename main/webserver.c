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

#include "webserver.h"

#include <esp_log.h>

#include <stdio.h>
#include <stdlib.h>

#define TAG "httpd"

#define MIN(a,b) ((a) < (b) ? (a) : (b))

static const char* insert_tracks(httpd_req_t *req, const char *page, int *len)
{
    const char tag[] = "<!-- %%TRACKS%% -->";
    const char *ctag = memmem(page, *len, tag, sizeof(tag) - 1);
    char buf[128];

    if (ctag) {
        const struct Track **tracks = (const struct Track**)req->user_ctx;
        int part_len = ctag - page;
        httpd_resp_send_chunk(req, (const char *)page, part_len);
        for (unsigned int i = 0; tracks[i]; i++) {
            httpd_resp_sendstr_chunk(req, "<h2>");
            httpd_resp_sendstr_chunk(req, tracks[i]->label);
            httpd_resp_sendstr_chunk(req, "</h2>\nTimings in ms.\n");
            snprintf(buf, sizeof(buf), "<form action=\"/api/track%d/timings/\" method=\"POST\">\n", i + 1);
            httpd_resp_sendstr_chunk(req, buf);
            httpd_resp_sendstr_chunk(req, "<table>\n");
            snprintf(buf, sizeof(buf), "<tr><th><label for=\"decDuration%d\">deceleration duration</label></th>", i);
            httpd_resp_sendstr_chunk(req, buf);
            snprintf(buf, sizeof(buf), "<th><label for=\"passingDuration%d\">passing duration</label></th>", i);
            httpd_resp_sendstr_chunk(req, buf);
            snprintf(buf, sizeof(buf), "<th><label for=\"stationDuration%d\">station duration</label></th>", i);
            httpd_resp_sendstr_chunk(req, buf);
            snprintf(buf, sizeof(buf), "<th><label for=\"breakDuration%d\">break duration</label></th>", i);
            httpd_resp_sendstr_chunk(req, buf);
            snprintf(buf, sizeof(buf), "<th><label for=\"stopDuration%d\">stop duration</label></th>", i);
            httpd_resp_sendstr_chunk(req, buf);
            snprintf(buf, sizeof(buf), "<th><label for=\"accDuration%d\">accelertion duration</label></th></tr>", i);
            httpd_resp_sendstr_chunk(req, buf);
            httpd_resp_sendstr_chunk(req, "<tr><td>");
            snprintf(buf, sizeof(buf), "<input type=\"number\" min=\"1000\" max=\"6000\" step=\"100\""
                     "id=\"decDuration\" name=\"decDuration%d\" value=\"%d\" />",
                     i, tracks[i]->timings.decDuration);
            httpd_resp_sendstr_chunk(req, buf);
            httpd_resp_sendstr_chunk(req, "</td>\n<td>");
            snprintf(buf, sizeof(buf), "<input type=\"number\" min=\"1000\" max=\"6000\" step=\"100\""
                     "id=\"passingDuration\" name=\"passingDuration%d\" value=\"%d\" />",
                     i, tracks[i]->timings.passingDuration);
            httpd_resp_sendstr_chunk(req, buf);
            httpd_resp_sendstr_chunk(req, "</td>\n<td>");
            snprintf(buf, sizeof(buf), "<input type=\"number\" min=\"1000\" max=\"6000\" step=\"100\""
                     "id=\"stationDuration\" name=\"stationDuration%d\" value=\"%d\" />",
                     i, tracks[i]->timings.stationDuration);
            httpd_resp_sendstr_chunk(req, buf);
            httpd_resp_sendstr_chunk(req, "</td>\n<td>");
            snprintf(buf, sizeof(buf), "<input type=\"number\" min=\"100\" max=\"1000\" step=\"100\""
                     "id=\"breakDuration\" name=\"breakDuration%d\" value=\"%d\" />",
                     i, tracks[i]->timings.breakDuration);
            httpd_resp_sendstr_chunk(req, buf);
            httpd_resp_sendstr_chunk(req, "</td>\n<td>");
            snprintf(buf, sizeof(buf), "<input type=\"number\" min=\"1000\" max=\"12000\" step=\"1000\""
                     "id=\"stopDuration\" name=\"stopDuration%d\" value=\"%d\" />",
                     i, tracks[i]->timings.stopDuration);
            httpd_resp_sendstr_chunk(req, buf);
            httpd_resp_sendstr_chunk(req, "</td>\n<td>");
            snprintf(buf, sizeof(buf), "<input type=\"number\" min=\"1000\" max=\"6000\" step=\"100\""
                     "id=\"accDuration\" name=\"accDuration%d\" value=\"%d\" />",
                     i, tracks[i]->timings.accDuration);
            httpd_resp_sendstr_chunk(req, buf);
            httpd_resp_sendstr_chunk(req, "</td></tr>\n");
            httpd_resp_sendstr_chunk(req, "</table>\n");
            httpd_resp_sendstr_chunk(req, "<input type=\"submit\" value=\"Update\">\n");
            httpd_resp_sendstr_chunk(req, "</form>\n");
        }
        *len -= part_len + sizeof(tag) - 1;
        return ctag + sizeof(tag) - 1;
    } else {
        return page;
    }
}

static esp_err_t index_get_handler(httpd_req_t *req)
{
    extern const unsigned char index_start[] asm("_binary_index_html_start");
    extern const unsigned char index_end[]   asm("_binary_index_html_end");
    int len = (index_end - index_start);

    ESP_LOGI(TAG, "%s", req->uri);
    httpd_resp_set_type(req, "text/html");

    const char *end = insert_tracks(req, (const char*)index_start, &len);
    httpd_resp_send_chunk(req, end, len);
    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

static esp_err_t timings_post_handler(httpd_req_t *req)
{
    char buf[256], *sep, *data;
    int ret, remaining = req->content_len, ln;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf) - 1))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }
        remaining -= ret;
        buf[ret] = '&';

        data = buf;
        ln = sizeof(buf);
        while (ln > 0 && (sep = memmem(data, ln, "&", 1)) != NULL) {
            int tln = sep - data;
            int value;

            *sep = '\0';
            sep = memmem(data, tln, "=", 1);
            if (sep) {
                *sep = '\0';
                value = atoi(sep + 1);
                ESP_LOGI(TAG, "%s: %s = %d", req->uri, data, value);
                if (!strcmp(data, "decDuration")) {
                    track_set_dec_duration((struct Track*)req->user_ctx, value);
                } else if (!strcmp(data, "passingDuration")) {
                    track_set_passing_duration((struct Track*)req->user_ctx, value);
                } else if (!strcmp(data, "stationDuration")) {
                    track_set_station_duration((struct Track*)req->user_ctx, value);
                } else if (!strcmp(data, "breakDuration")) {
                    track_set_break_duration((struct Track*)req->user_ctx, value);
                } else if (!strcmp(data, "stopDuration")) {
                    track_set_stop_duration((struct Track*)req->user_ctx, value);
                } else if (!strcmp(data, "accDuration")) {
                    track_set_acc_duration((struct Track*)req->user_ctx, value);
                }
            }

            data += tln + 1;
            ln -= tln + 1;
        }
    }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_sendstr(req, "Timings updated successfully");
    return ESP_OK;
}

static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    extern const unsigned char fav_start[] asm("_binary_favicon_png_start");
    extern const unsigned char fav_end[]   asm("_binary_favicon_png_end");
    const size_t fav_size = (fav_end - fav_start);
    ESP_LOGI(TAG, "%s: size %ld", req->uri, fav_size);
    httpd_resp_set_type(req, "image/png");
    httpd_resp_send(req, (const char *)fav_start, fav_size);
    return ESP_OK;
}

static httpd_uri_t trainControl = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_get_handler,
    .user_ctx  = NULL
};

static httpd_uri_t track1Timings = {
    .uri       = "/api/track1/timings/",
    .method    = HTTP_POST,
    .handler   = timings_post_handler,
    .user_ctx  = NULL
};

static httpd_uri_t track2Timings = {
    .uri       = "/api/track2/timings/",
    .method    = HTTP_POST,
    .handler   = timings_post_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t favicon = {
    .uri       = "/favicon.png",
    .method    = HTTP_GET,
    .handler   = favicon_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t favico = {
    .uri       = "/favicon.ico",
    .method    = HTTP_GET,
    .handler   = favicon_get_handler,
    .user_ctx  = NULL
};

static esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Unknown URL on this server.");
    return ESP_OK;
}

httpd_handle_t start_webserver(const struct Track *tracks[])
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 20480;
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        trainControl.user_ctx = tracks;
        httpd_register_uri_handler(server, &trainControl);
        if (tracks[0]) {
            track1Timings.user_ctx = (void*)tracks[0];
            httpd_register_uri_handler(server, &track1Timings);
            if (tracks[1]) {
                track2Timings.user_ctx = (void*)tracks[1];
                httpd_register_uri_handler(server, &track2Timings);
            }
        }
        httpd_register_uri_handler(server, &favicon);
        httpd_register_uri_handler(server, &favico);
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t *server)
{
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        if (httpd_stop(*server) == ESP_OK) {
            *server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}
