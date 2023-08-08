#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_sdp_api.h"
#include "esp_spp_api.h"

#define MAX_SPP_CLIENTS 5
#define SPP_NO_CLIENT 65535

struct BtPayload {
    int len;
    uint8_t *data;
};

struct BtCommand {
    uint32_t handle;
    int speed;
};

struct SppClient {
    uint32_t handle;
    struct BtPayload payload;
    int alive;
};
static struct SppClient sppClients[MAX_SPP_CLIENTS];

struct BtPayload* bt_payload(uint32_t handle)
{
    for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
        if (sppClients[i].handle == handle) {
            return &sppClients[i].payload;
        }
    }
    ESP_LOGI("Bluetooth", "unknown handle: %ld", handle);
    return 0;
}

#define MAX_SPP_SOURCE 256
struct BtSource {
    int len;
    uint8_t data[MAX_SPP_SOURCE];
};
static struct BtSource capabilitiesFrame;
static gptimer_handle_t pingTimer = NULL;

#define MAX_TRACKS 2
struct Track;
static struct Track *tracks[MAX_TRACKS];

void bt_register_track(int trackId, struct Track *track)
{
    if (trackId < 0 || trackId >= MAX_TRACKS)
        return;

    tracks[trackId] = track;
    ESP_LOGI("Bluetooth", "registering track %d: %p", trackId, tracks[trackId]);
}
int track_id(struct Track *track);
uint32_t track_is_bluetooth_controlled(const struct Track *track);
void track_set_bluetooth_controlled(struct Track *track, uint32_t handle);
void track_set_bluetooth_speed(struct Track *track, int speed);

int bt_source_equals(struct BtSource *a, struct BtSource *b)
{
    if (a->len != b->len) {
        return 0;
    }

    for (int i = 0; i < a->len; i++) {
        if (a->data[i] != b->data[i]) {
            return 0;
        }
    }

    return 1;
}

void bt_spp_send(struct SppClient *spp, struct BtSource *source)
{
    if (!spp->payload.len) {
        spp->payload.data = source->data;
        spp->payload.len = source->len;
        ESP_ERROR_CHECK(esp_spp_write(spp->handle, source->len, source->data));
    } else {
        ESP_LOGI("Bluetooth", "dropping frame.");
    }
}

void bt_source_send_all(struct BtSource *source)
{
    for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
        if (sppClients[i].handle != SPP_NO_CLIENT) {
            bt_spp_send(sppClients + i, source);
        }
    }
}

void bt_source_send_new(uint32_t handle, struct BtSource *source)
{
    for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
        if (sppClients[i].handle == SPP_NO_CLIENT) {
            sppClients[i].handle = handle;
            sppClients[i].alive = 1;
            sppClients[i].payload.len = 0;
            bt_spp_send(sppClients + i, source);
            return;
        }
    }
}

enum FrameType {
                UNSUPPORTED,
                PING,
                CAPABILITIES,
                TRACK_STATE,
                ACQUIRE_TRACK,
                ACQUIRE_ACK,
                RELEASE_TRACK,
                RELEASE_ACK,
                SPEED_COMMAND
};

int bt_copy(struct BtSource *at, uint8_t *data, int len)
{
    if (at->len + len < MAX_SPP_SOURCE) {
        for (int i = 0; i < len; i++) {
            at->data[at->len + i] = data[i];
        }
        at->len += len;
        return len;
    } else {
        return -1;
    }
}
int bt_pack_start(struct BtSource *at, enum FrameType type)
{
    at->len = 0;
    return bt_copy(at, (uint8_t*)&type, sizeof(enum FrameType) / sizeof(uint8_t));
}
int bt_pack_int(struct BtSource *at, int value)
{
    return bt_copy(at, (uint8_t*)&value, sizeof(int) / sizeof(uint8_t));
}
int bt_pack_int16(struct BtSource *at, uint16_t value)
{
    return bt_copy(at, (uint8_t*)&value, sizeof(uint16_t) / sizeof(uint8_t));
}
int bt_pack_uint64(struct BtSource *at, uint64_t value)
{
    return bt_copy(at, (uint8_t*)&value, sizeof(uint64_t) / sizeof(uint8_t));
}
int bt_pack_str(struct BtSource *at, const char *str)
{
    return bt_copy(at, (uint8_t*)str, (strlen(str) + 1) * sizeof(char) / sizeof(uint8_t));
}

uint8_t* bt_get_frame_data(struct BtPayload *payload,
                           enum FrameType type, int typeLen)
{
    if (!payload || payload->len < typeLen)
        return (uint8_t*)0;
    enum FrameType tp = *(enum FrameType*)payload->data;
    if (type == tp) {
        ESP_LOGI("Bluetooth", "frame received: %d", tp);
        uint8_t *dt = payload->data + sizeof(enum FrameType);
        payload->data += typeLen;
        payload->len -= typeLen;
        return dt;
    } else {
        return (uint8_t*)0;
    }
}

void bt_get_frame_track(uint8_t **frame, int *trackId, struct Track **track)
{
    if (track && trackId) {
        *trackId = *(int*)*frame;
        if (*trackId >= 0 && *trackId < MAX_TRACKS)
            *track = tracks[*trackId];
        else
            *track = (struct Track*)0;
    }
    *frame += sizeof(uint32_t);
}

void bt_send_ping_frame(uint64_t count)
{
    struct BtSource pingFrame;
    bt_pack_start(&pingFrame, PING);
    bt_pack_uint64(&pingFrame, count);
    for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
        if (sppClients[i].handle != SPP_NO_CLIENT && sppClients[i].alive) {
            if (!sppClients[i].payload.len) {
                ESP_LOGI("Bluetooth", "sending ping frame: %lld", count);
                sppClients[i].payload.data = pingFrame.data;
                sppClients[i].payload.len = pingFrame.len;
                ESP_ERROR_CHECK(esp_spp_write(sppClients[i].handle,
                                              pingFrame.len, pingFrame.data));
                sppClients[i].alive = 0;
            } else {
                ESP_LOGI("Bluetooth", "dropping ping frame.");
            }
        } else if (sppClients[i].handle != SPP_NO_CLIENT) {
            ESP_LOGI("Bluetooth", "client did not respond, disconnecting.");
            ESP_ERROR_CHECK(esp_spp_disconnect(sppClients[i].handle));
        }
    }
}

void bt_receive_ping_frame(uint32_t handle, uint64_t count)
{
    ESP_LOGI("Bluetooth", "ping response: %lld (%ld)", count, handle);
    for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
        if (sppClients[i].handle == handle && !sppClients[i].alive) {
            sppClients[i].alive = 1;
        }
    }
}

int isPingFrame(struct BtPayload *payload, uint64_t *count)
{
    static int pingLen = sizeof(enum FrameType) + sizeof(uint64_t);

    const uint8_t *frame = bt_get_frame_data(payload, PING, pingLen);
    if (frame && count) {
        *count = *(uint64_t*)frame;
    }
    return (frame != (uint8_t*)0);
}

void bt_receive_acquire_frame(uint32_t handle, int trackId, struct Track *track)
{
    int acquire = track && (track_is_bluetooth_controlled(track) == SPP_NO_CLIENT);
    struct BtSource ack;
    bt_pack_start(&ack, ACQUIRE_ACK);
    bt_pack_int(&ack, trackId);
    bt_pack_int(&ack, acquire);
    ESP_LOGI("Bluetooth", "acquire response for track %d: %d", trackId, acquire);
    if (track && acquire)
        track_set_bluetooth_controlled(track, handle);

    for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
        if (sppClients[i].handle == handle) {
            bt_spp_send(sppClients + i, &ack);
            return;
        }
    }
}

int isAcquireFrame(struct BtPayload *payload, int *trackId, struct Track **track)
{
    static int acquireLen = sizeof(enum FrameType) + sizeof(uint32_t);

    uint8_t *frame = bt_get_frame_data(payload, ACQUIRE_TRACK, acquireLen);
    if (frame) {
        bt_get_frame_track(&frame, trackId, track);
    }
    return (frame != (uint8_t*)0);
}

void bt_receive_release_frame(uint32_t handle, int trackId, struct Track *track)
{
    int release = track && (track_is_bluetooth_controlled(track) == handle);
    struct BtSource ack;
    bt_pack_start(&ack, RELEASE_ACK);
    bt_pack_int(&ack, trackId);
    bt_pack_int(&ack, release);
    ESP_LOGI("Bluetooth", "release response for track %d: %d", trackId, release);
    if (track && release)
        track_set_bluetooth_controlled(track, SPP_NO_CLIENT);

    for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
        if (sppClients[i].handle == handle) {
            bt_spp_send(sppClients + i, &ack);
            return;
        }
    }
}

int isReleaseFrame(struct BtPayload *payload, int *trackId, struct Track **track)
{
    static int releaseLen = sizeof(enum FrameType) + sizeof(uint32_t);

    uint8_t *frame = bt_get_frame_data(payload, RELEASE_TRACK, releaseLen);
    if (frame) {
        bt_get_frame_track(&frame, trackId, track);
    }
    return (frame != (uint8_t*)0);
}

int isSpeedFrame(struct BtPayload *payload,
                 int *trackId, struct Track **track, int *speed)
{
    static int speedLen = sizeof(enum FrameType) + sizeof(uint32_t) + sizeof(int32_t);

    uint8_t *frame = bt_get_frame_data(payload, SPEED_COMMAND, speedLen);
    if (frame) {
        bt_get_frame_track(&frame, trackId, track);
        if (speed) {
            *speed = *(int32_t*)frame;
        }
    }
    return (frame != (uint8_t*)0);
}

static const uint8_t UUID_SPP[] = {0x00, 0x00, 0x11, 0x01, 0x00, 0x00, 0x10, 0x00,
                                   0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
static char *sdp_service_name = "Train supervision";
static void bt_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case (ESP_SPP_INIT_EVT):
        ESP_LOGI("Bluetooth", "SPP initialisation status: %d", param->init.status);
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_ERROR_CHECK(esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE,
                                              0, "Train supervisor"));
        }
        break;
    case (ESP_SPP_UNINIT_EVT):
        ESP_LOGI("Bluetooth", "SPP finalisation status: %d", param->uninit.status);
        break;
    case (ESP_SPP_START_EVT): {
        esp_bluetooth_sdp_record_t record = {0};
        ESP_LOGI("Bluetooth", "SPP server started status: %d, channel: %d",
                 param->start.status, param->start.scn);
        record.hdr.type = ESP_SDP_TYPE_RAW;
        record.hdr.uuid.len = sizeof(UUID_SPP);
        memcpy(record.hdr.uuid.uuid.uuid128, UUID_SPP, sizeof(UUID_SPP));
        record.hdr.service_name_length = strlen(sdp_service_name) + 1;
        record.hdr.service_name = sdp_service_name;
        record.hdr.rfcomm_channel_number = param->start.scn;
        record.hdr.l2cap_psm = -1;
        record.hdr.profile_version = 1;
        esp_sdp_create_record(&record);

        for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
            sppClients[i].handle = SPP_NO_CLIENT;
            sppClients[i].payload.len = 0;
            sppClients[i].payload.data = 0;
        }
        break;
    }
    case (ESP_SPP_CLOSE_EVT):
        ESP_LOGI("Bluetooth", "SPP server disconnection status: %d", param->close.status);
        for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
            if (sppClients[i].handle == param->close.handle) {
                sppClients[i].handle = SPP_NO_CLIENT;
                sppClients[i].payload.len = 0;
                sppClients[i].payload.data = 0;
                break;
            }
        }
        for (int i = 0; i < MAX_TRACKS; i++) {
            if (tracks[i] && (track_is_bluetooth_controlled(tracks[i]) == param->close.handle)) {
                track_set_bluetooth_controlled(tracks[i], SPP_NO_CLIENT);
            }
        }
        int anyClient = 0;
        for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
            if (sppClients[i].handle != SPP_NO_CLIENT)
                anyClient = 1;
        }
        if (!anyClient) {
            ESP_ERROR_CHECK(gptimer_stop(pingTimer));
        }
        break;
    case (ESP_SPP_SRV_OPEN_EVT):
        ESP_LOGI("Bluetooth", "SPP server connection status: %d, address: "ESP_BD_ADDR_STR,
                 param->srv_open.status, param->srv_open.rem_bda[0], param->srv_open.rem_bda[1],
                 param->srv_open.rem_bda[2], param->srv_open.rem_bda[3],
                 param->srv_open.rem_bda[4], param->srv_open.rem_bda[5]);
        bt_source_send_new(param->srv_open.handle, &capabilitiesFrame);
        ESP_ERROR_CHECK(gptimer_start(pingTimer));
        break;
    case (ESP_SPP_SRV_STOP_EVT):
        ESP_LOGI("Bluetooth", "SPP server disconnection status: %d, channel: %d",
                 param->srv_stop.status, param->srv_stop.scn);
        break;
    case (ESP_SPP_DATA_IND_EVT):
        ESP_LOGI("Bluetooth", "SPP read status: %d, length: %d",
                 param->data_ind.status, param->data_ind.len);
        if (param->data_ind.status == ESP_SPP_SUCCESS) {
            uint64_t ping;
            int trackId, speed;
            struct Track *track;
            struct BtPayload payload;
            payload.data = param->data_ind.data;
            payload.len = param->data_ind.len;
            while (payload.len > 0) {
                if (isPingFrame(&payload, &ping)) {
                    bt_receive_ping_frame(param->data_ind.handle, ping);
                } else if (isAcquireFrame(&payload, &trackId, &track)) {
                    bt_receive_acquire_frame(param->data_ind.handle, trackId, track);
                } else if (isReleaseFrame(&payload, &trackId, &track)) {
                    bt_receive_release_frame(param->data_ind.handle, trackId, track);
                } else if (isSpeedFrame(&payload, &trackId, &track, &speed) && track) {
                    track_set_bluetooth_speed(track, speed);
                } else {
                    ESP_LOGI("Bluetooth", "SPP read error, remains %d data.", payload.len);
                    payload.len = 0;
                }
            }
        }
        break;
    case (ESP_SPP_CONG_EVT): {
        ESP_LOGI("Bluetooth", "SPP congestion status: %d, congestion: %d",
                 param->cong.status, param->cong.cong);
        struct BtPayload *payload = bt_payload(param->cong.handle);
        if (payload && !param->cong.cong && payload->len) {
            ESP_ERROR_CHECK(esp_spp_write(param->cong.handle,
                                          payload->len,
                                          payload->data));
        }
        break;
    }
    case (ESP_SPP_WRITE_EVT): {
        ESP_LOGI("Bluetooth", "SPP write status: %d, length: %d",
                 param->write.status, param->write.len);
        struct BtPayload *payload = bt_payload(param->write.handle);
        if (payload) {
            if (param->write.len < payload->len) {
                if (!param->write.cong) {
                    payload->len -= param->write.len;
                    payload->data += param->write.len;
                    ESP_ERROR_CHECK(esp_spp_write(param->write.handle,
                                                  payload->len,
                                                  payload->data));
                } else {
                    ESP_LOGI("Bluetooth", "SPP write congestion.");
                }
            } else {
                payload->len = 0;
                payload->data = 0;
            }
        }
        break;
    }
    default:
        break;
    }
}

static void sdp_callback(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param)
{
    switch (event) {
    case ESP_SDP_INIT_EVT:
        ESP_LOGI("Bluetooth", "SDP initialisation status: %d", param->init.status);
        break;
    case ESP_SDP_DEINIT_EVT:
    case ESP_SDP_SEARCH_COMP_EVT:
    case ESP_SDP_CREATE_RECORD_COMP_EVT: {
        ESP_LOGI("Bluetooth", "SDP new record status: %d", param->create_record.status);
        if (param->create_record.status == ESP_SDP_SUCCESS) {
            esp_bt_dev_set_device_name("ESP train");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }
    }
    case ESP_SDP_REMOVE_RECORD_COMP_EVT:
        break;
    default:
        ESP_LOGI("Bluetooth", "Invalid SDP event: %d", event);
        break;
    }
}

struct System {
    adc_oneshot_unit_handle_t adc;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t pwm;
    QueueHandle_t pingQueue;
};

static bool IRAM_ATTR system_ping_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;
    static uint64_t count = 0;
    xQueueSendFromISR(((struct System*)user_ctx)->pingQueue, &count, &high_task_awoken);
    count++;
    return high_task_awoken == pdTRUE;
}

void system_new(struct System *system)
{
    adc_oneshot_unit_init_cfg_t adcCfg =
        {
         .unit_id = ADC_UNIT_1,
         .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adcCfg, &system->adc));

    mcpwm_timer_config_t timerCfg =
        {
         .group_id = 0,
         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
         .resolution_hz = 409600,
         .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
         .period_ticks = 4096,
        };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timerCfg, &system->timer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(system->timer));

    mcpwm_operator_config_t operatorCfg = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operatorCfg, &system->pwm));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(system->pwm, system->timer));

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    
    esp_bt_controller_config_t btCfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&btCfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_sdp_register_callback(sdp_callback));
    ESP_ERROR_CHECK(esp_sdp_init());

    ESP_ERROR_CHECK(esp_spp_register_callback(bt_callback));
    esp_spp_cfg_t sppCfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = 0,
        .tx_buffer_size = 0
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&sppCfg));

    for (int i = 0; i < MAX_TRACKS; i++)
        tracks[i] = (struct Track*)0;

    system->pingQueue = xQueueCreate(1, sizeof(uint64_t));
    gptimer_config_t pingCfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 10000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&pingCfg, &pingTimer));
    gptimer_event_callbacks_t pingCallback = {
        .on_alarm = system_ping_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(pingTimer, &pingCallback, system));
    gptimer_alarm_config_t alarmCfg = {
        .reload_count = 0, // counter will reload with 0 on alarm event
        .alarm_count = 10000, // period = 1s @resolution 10kHz
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_enable(pingTimer));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(pingTimer, &alarmCfg));
};

void system_free(struct System *system)
{
    ESP_ERROR_CHECK(adc_oneshot_del_unit(system->adc));
    ESP_ERROR_CHECK(mcpwm_del_operator(system->pwm));
    ESP_ERROR_CHECK(mcpwm_del_timer(system->timer));
    ESP_ERROR_CHECK(gptimer_stop(pingTimer));
    ESP_ERROR_CHECK(gptimer_disable(pingTimer));
    ESP_ERROR_CHECK(gptimer_del_timer(pingTimer));
    ESP_ERROR_CHECK(esp_spp_stop_srv());
    ESP_ERROR_CHECK(esp_spp_deinit());
    ESP_ERROR_CHECK(esp_bluedroid_disable());
    ESP_ERROR_CHECK(esp_bluedroid_deinit());
    ESP_ERROR_CHECK(esp_bt_controller_disable());
    ESP_ERROR_CHECK(esp_bt_controller_deinit());
    if (system->pingQueue) vQueueDelete(system->pingQueue);
}

void system_start(struct System *system)
{
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(system->timer, MCPWM_TIMER_START_NO_STOP));
}

struct SpeedInput {
    adc_channel_t variablePin;
    gpio_num_t forwardPin, backwardPin;
};

void speed_input_setup(const struct SpeedInput *command, const struct System *system)
{
    adc_oneshot_chan_cfg_t config =
        {
         .bitwidth = ADC_BITWIDTH_9,
         .atten = ADC_ATTEN_DB_11,
        };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(system->adc, command->variablePin, &config));

    ESP_ERROR_CHECK(gpio_set_direction(command->forwardPin, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(command->forwardPin, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_pullup_en(command->forwardPin));
    ESP_ERROR_CHECK(gpio_set_direction(command->backwardPin, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(command->backwardPin, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_pullup_en(command->backwardPin));
}

struct PWMOutput {
    gpio_num_t enable, pwm1, pwm2;
    mcpwm_cmpr_handle_t compare;
    mcpwm_gen_handle_t generate;
};

void pwm_output_setup(struct PWMOutput *pwm, const struct System *system)
{
    mcpwm_comparator_config_t compareCfg = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(system->pwm, &compareCfg, &pwm->compare));
    mcpwm_generator_config_t genCfg =
        {
         .gen_gpio_num = pwm->enable,
        };
    ESP_ERROR_CHECK(mcpwm_new_generator(system->pwm, &genCfg, &pwm->generate));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event
                    (pwm->generate,
                     MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                  MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event
                    (pwm->generate,
                     MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                    pwm->compare, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(gpio_set_direction(pwm->pwm1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pwm->pwm1, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pwm->pwm2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pwm->pwm2, 0));
}

struct PositionInput {
    gpio_num_t close1, close2;
    gpio_num_t far1, far2;
};

enum Positions {
                CLOSE_1,
                CLOSE_2,
                FAR_1,
                FAR_2
};

void position_input_setup(const struct PositionInput *positions)
{
    if (positions->close1 != GPIO_NUM_NC)
        ESP_ERROR_CHECK(gpio_set_direction(positions->close1, GPIO_MODE_INPUT));
    if (positions->close2 != GPIO_NUM_NC)
        ESP_ERROR_CHECK(gpio_set_direction(positions->close2, GPIO_MODE_INPUT));
    if (positions->far1 != GPIO_NUM_NC)
        ESP_ERROR_CHECK(gpio_set_direction(positions->far1, GPIO_MODE_INPUT));
    if (positions->far2 != GPIO_NUM_NC)
        ESP_ERROR_CHECK(gpio_set_direction(positions->far2, GPIO_MODE_INPUT));
}

int position_input_isTriggered(const struct PositionInput *positions, enum Positions position)
{
    switch (position) {
    case CLOSE_1:
        return (positions->close1 != GPIO_NUM_NC) ? !gpio_get_level(positions->close1) : 0;
    case CLOSE_2:
        return (positions->close2 != GPIO_NUM_NC) ? !gpio_get_level(positions->close2) : 0;
    case FAR_1:
        return (positions->far1 != GPIO_NUM_NC) ? !gpio_get_level(positions->far1) : 0;
    case FAR_2:
        return (positions->far2 != GPIO_NUM_NC) ? !gpio_get_level(positions->far2) : 0;
    default:
        return 0;
    }
}

void position_input_log(const struct PositionInput *positions)
{
    if (position_input_isTriggered(positions, FAR_1))
        ESP_LOGI("Position", "FAR_1");
    if (position_input_isTriggered(positions, CLOSE_1))
        ESP_LOGI("Position", "CLOSE_1");
    if (position_input_isTriggered(positions, CLOSE_2))
        ESP_LOGI("Position", "CLOSE_2");
    if (position_input_isTriggered(positions, FAR_2))
        ESP_LOGI("Position", "FAR_2");
}

int position_input_isEnabled(const struct PositionInput *positions)
{
    return (positions->close1 != GPIO_NUM_NC)
        || (positions->close2 != GPIO_NUM_NC)
        || (positions->far1 != GPIO_NUM_NC)
        || (positions->far2 != GPIO_NUM_NC);
}

enum Source {
             HARDWARE,
             BLUETOOTH
};

struct HardwareCommand {
    const struct System *system;
    struct SpeedInput speed;
};

struct Command {
    enum Source source;
    union {
        struct HardwareCommand hd;
        struct BtCommand bt;
    };
    int speed;
    int isForward, isBackward;
};

void command_init_hardware(struct Command *command, const struct System *system, struct SpeedInput speed)
{
    command->source = HARDWARE;
    command->hd.system = system;
    command->hd.speed = speed;
    speed_input_setup(&command->hd.speed, system);
}

void command_init_bluetooth(struct Command *command, uint32_t handle, const struct Command *src)
{
    *command = *src;
    command->source = BLUETOOTH;
    command->bt.handle = handle;
    command->bt.speed = command->isForward ? command->speed : -command->speed;
}

void command_update(struct Command *command)
{
    switch (command->source) {
    case (HARDWARE):
        ESP_ERROR_CHECK(adc_oneshot_read(command->hd.system->adc, command->hd.speed.variablePin, &command->speed));
        command->speed = command->speed ? 512 + 7 * command->speed : 0;
        command->isForward = !gpio_get_level(command->hd.speed.forwardPin);
        command->isBackward = !gpio_get_level(command->hd.speed.backwardPin);
        break;
    case (BLUETOOTH):
        command->speed = command->bt.speed > 0 ? command->bt.speed : -command->bt.speed;
        command->isForward = command->bt.speed > 0;
        command->isBackward = command->bt.speed < 0;
        break;
    default:
        return;
    }
    command->isForward = command->isForward && command->speed; 
    command->isBackward = command->isBackward && command->speed; 
}

struct Timings {
    int sleepTime;

    int passingSpeed, stationSpeed;
    int passingDuration, stationDuration;
    int decDuration;
    int decTarget, accDuration, breakDuration;

    int stopDuration, stopCount;
};

void timings_adjust_speed(int *speed, int targetDuration, int realDuration, int maxSpeed)
{
    *speed = *speed * realDuration / targetDuration;
    if (*speed < 768)
        *speed = 768;
    if (*speed > maxSpeed)
        *speed = maxSpeed;
}

void timings_adjust_duration(int *duration, int targetDuration, int realDuration)
{
    *duration = *duration * realDuration / targetDuration;
    if (*duration < 300)
        *duration = 300;
    if (*duration > 5000)
        *duration = 5000;
}

enum States {
             SOMEWHERE,
             APPROACHING,
             PASSING_BY,
             STOPPING,
             IN_STATION,
             LEAVING
};

int bt_pack_track_state(struct BtSource *at, enum States state)
{
    return bt_copy(at, (uint8_t*)&state, sizeof(enum States) / sizeof(uint8_t));
}

enum Capabilities {
                   SPEED_CONTROL = 1,
                   POSITIONING   = 2
};

struct Track {
    int id;
    const char* label;

    const struct System *system;
    struct Command hdCommand, btCommand;
    struct Command *command;
    struct PWMOutput pwm;
    struct PositionInput positions;
    struct Timings timings;

    int speed;
    int duration, elapsed;
    /* gptimer_handle_t timer; */
    int count;
    enum States state;

    struct BtSource stateFrame;
};

#define MAX_SPEED 4095     // Maximum speed value

void track_new(struct Track *track, const char *label,
               const struct System *system,
               const struct SpeedInput input,
               const struct PWMOutput pwm,
               const struct PositionInput positions,
               const struct Timings timings)
{
    static int id = 0;

    track->id = id++;
    track->label = label;

    track->system = system;

    track->command = &track->hdCommand;
    command_init_hardware(&track->hdCommand, system, input);
    bt_register_track(track->id, track);

    track->pwm = pwm;
    pwm_output_setup(&track->pwm, system);

    track->positions = positions;
    position_input_setup(&track->positions);

    track->timings = timings;

    track->speed = 0;
    track->duration = 0;
    track->count = 0;
    track->state = SOMEWHERE;
}

void track_free(struct Track *track)
{
    ESP_ERROR_CHECK(mcpwm_del_generator(track->pwm.generate));
    ESP_ERROR_CHECK(mcpwm_del_comparator(track->pwm.compare));
}

int track_id(struct Track *track)
{
    return track ? track->id : -1;
}

uint32_t track_is_bluetooth_controlled(const struct Track *track)
{
    return (track && (track->command == &track->btCommand)) ? track->btCommand.bt.handle : SPP_NO_CLIENT;
}

void track_set_bluetooth_controlled(struct Track *track, uint32_t handle)
{
    if (handle == SPP_NO_CLIENT) {
        track->command = &track->hdCommand;
    } else {
        track->command = &track->btCommand;
        command_init_bluetooth(&track->btCommand, handle, &track->hdCommand);
    }
}

void track_set_bluetooth_speed(struct Track *track, int speed)
{
    track->btCommand.bt.speed = speed;
}

void track_setup_capabilities(struct Track *track)
{
    bt_pack_int(&capabilitiesFrame, track->id);
    bt_pack_int(&capabilitiesFrame, strlen(track->label));
    bt_pack_str(&capabilitiesFrame, track->label);
    bt_pack_int(&capabilitiesFrame, MAX_SPEED);
    uint16_t flag = SPEED_CONTROL;
    if (position_input_isEnabled(&track->positions)) {
        flag |= POSITIONING;
    }
    bt_pack_int16(&capabilitiesFrame, flag);
}

#define AUTO_DETECT 0      // No timer, next transition is based on detection
void track_set_state(struct Track *track, enum States state)
{
    switch (state) {
    case SOMEWHERE:
        track->duration = AUTO_DETECT;
        break;
    case APPROACHING:
        track->duration = track->timings.decDuration;
        track->elapsed = 0;
        break;
    case PASSING_BY:
        if (track->state == APPROACHING) {
            timings_adjust_duration(&track->timings.decDuration,
                                    track->timings.decTarget, track->elapsed);
        }
        track->duration = AUTO_DETECT;
        track->elapsed = 0;
        break;
    case LEAVING:
        if (track->state == PASSING_BY) {
            timings_adjust_speed(&track->timings.passingSpeed,
                                 track->timings.passingDuration, track->elapsed, 3072);
        }
        track->duration = track->timings.accDuration;
        track->elapsed = -1;
        break;
    case STOPPING:
        if (track->state == PASSING_BY) {
            timings_adjust_speed(&track->timings.stationSpeed,
                                 track->timings.stationDuration, track->elapsed, 2048);
        }
        track->duration = track->timings.breakDuration;
        track->elapsed = -1;
        break;
    case IN_STATION:
        track->duration = track->timings.stopDuration;
        break;
    }
    track->state = state;
}

int track_state_is_done(const struct Track *track)
{
    return (track->duration <= 0);
}

int track_state_get_speed_target(struct Track *track)
{
    switch (track->state) {
    case (APPROACHING):
    case (PASSING_BY): {
        int speedTarget = track->count % track->timings.stopCount
            ? track->timings.passingSpeed : track->timings.stationSpeed;
        int durationTarget = track->state == APPROACHING ? track->timings.decTarget
            : (track->count % track->timings.stopCount
               ? track->timings.passingDuration : track->timings.stationDuration);
        int delta = track->elapsed > durationTarget ? (track->elapsed - durationTarget) / 1000 : 0;
        return speedTarget + 64 * delta;
    }
    case (STOPPING):
    case (IN_STATION):
        return 0;
    default:
        return MAX_SPEED;
    }
}

int track_state_adjust_speed(struct Track *track, int value)
{
    int target = track_state_get_speed_target(track);
    target = value < target ? value : target;
    if (track->duration > 0) {
        int rate;
        rate = (target - track->speed) * track->timings.sleepTime / track->duration;
        return track->speed + rate < 0 ? 0 : (track->speed + rate > MAX_SPEED ? MAX_SPEED : track->speed + rate);
    } else {
        return target;
    }
}

int track_update_frame(struct Track *track)
{
    struct BtSource old = track->stateFrame;

    bt_pack_start(&track->stateFrame, TRACK_STATE);
    bt_pack_int(&track->stateFrame, track->id);
    bt_pack_int(&track->stateFrame, track->command->isForward);
    bt_pack_int(&track->stateFrame, track->command->isBackward);
    bt_pack_int(&track->stateFrame, track->speed);
    bt_pack_int(&track->stateFrame, track->count);
    bt_pack_track_state(&track->stateFrame, track->state);

    return !bt_source_equals(&old, &track->stateFrame);
}

int track_update(struct Track *track)
{
    command_update(track->command);

    position_input_log(&track->positions);
    if (track->state == SOMEWHERE
        && ((position_input_isTriggered(&track->positions, FAR_1) && track->command->isForward) ||
            (position_input_isTriggered(&track->positions, FAR_2) && track->command->isBackward))) {
        track_set_state(track, APPROACHING);
        ESP_LOGI("Position", "%s: train approaching", track->label);
    } else if (track->state == APPROACHING
               && ((position_input_isTriggered(&track->positions, CLOSE_1) && track->command->isForward) ||
                   (position_input_isTriggered(&track->positions, CLOSE_2) && track->command->isBackward))) {
        track_set_state(track, PASSING_BY);
        ESP_LOGI("Position", "%s: train breaks for the station in %d ms", track->label, track->elapsed);
        ESP_LOGI("Position", "%s: adjust approaching duration %d", track->label, track->timings.decDuration);
        ESP_LOGI("Position", "%s: train passing by the station (%d)", track->label, track->count);
    } else if (track->state == PASSING_BY
               && ((position_input_isTriggered(&track->positions, CLOSE_2) && track->command->isForward) ||
                   (position_input_isTriggered(&track->positions, CLOSE_1) && track->command->isBackward))) {
        if (track->count % track->timings.stopCount) {
            track_set_state(track, LEAVING);
            ESP_LOGI("Position", "%s: adjust passing speed %d", track->label, track->timings.passingSpeed);
        } else {
            track_set_state(track, STOPPING);
            ESP_LOGI("Position", "%s: adjust station speed %d", track->label, track->timings.stationSpeed);
        }
        ESP_LOGI("Position", "%s: train at platform end (in %d ms)", track->label, track->elapsed);
    } else if (track->state == STOPPING && !track->speed) {
        track_set_state(track, IN_STATION);
        ESP_LOGI("Position", "%s: train is stopped", track->label);
    } else if (track->state == IN_STATION && track_state_is_done(track)) {
        track_set_state(track, LEAVING);
        ESP_LOGI("Position", "%s: train is departing", track->label);
    } else if (track->state == LEAVING && track->speed >= track->command->speed) {
        track_set_state(track, SOMEWHERE);
        track->count += 1;
        ESP_LOGI("Position", "%s: train has done %d passings", track->label, track->count);
    }

    if (track->duration > 0 && (track->command->isBackward || track->command->isForward))
        track->duration -= track->timings.sleepTime;
    if (track->elapsed >= 0 && (track->command->isBackward || track->command->isForward))
        track->elapsed += track->timings.sleepTime;
    
    int isUpdated = 0;
    ESP_ERROR_CHECK(gpio_set_level(track->pwm.pwm1, track->command->isForward ? 1 : 0));
    ESP_ERROR_CHECK(gpio_set_level(track->pwm.pwm2, track->command->isBackward ? 1 : 0));

    int value = track_state_adjust_speed(track, track->command->speed);
    if (value != track->speed) {
        isUpdated = (value >> 7 != track->speed >> 7);
        track->speed = value;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(track->pwm.compare, value));
    }
    if (isUpdated) {
        ESP_LOGI("Power", "%s: PWM updated value: %d", track->label, value);
    }

    if (track_update_frame(track)) {
        bt_source_send_all(&track->stateFrame);
    }
    
    return isUpdated;
}

void esp1_set_pin_layout(struct Track *trackA, struct Track *trackB,
                         const struct System *system, const struct Timings timings)
{
    struct SpeedInput command;
    struct PWMOutput pwm;
    struct PositionInput positions;
    command.variablePin = ADC_CHANNEL_4;
    command.forwardPin  = GPIO_NUM_25;
    command.backwardPin = GPIO_NUM_26;
    pwm.enable = GPIO_NUM_5;
    pwm.pwm1 = GPIO_NUM_19;
    pwm.pwm2 = GPIO_NUM_18;
    positions.far1 = GPIO_NUM_35;
    positions.far2 = GPIO_NUM_36;
    positions.close1 = GPIO_NUM_34;
    positions.close2 = GPIO_NUM_39;
    track_new(trackA, "long track", system, command, pwm, positions, timings);
    command.variablePin = ADC_CHANNEL_5;
    command.forwardPin  = GPIO_NUM_27;
    command.backwardPin = GPIO_NUM_14;
    pwm.enable = GPIO_NUM_4;
    pwm.pwm1 = GPIO_NUM_17;
    pwm.pwm2 = GPIO_NUM_16;
    positions.far1 = GPIO_NUM_NC;
    positions.far2 = GPIO_NUM_NC;
    positions.close1 = GPIO_NUM_NC;
    positions.close2 = GPIO_NUM_NC;
    track_new(trackB, "small track", system, command, pwm, positions, timings);
}

void esp2_set_pin_layout(struct Track *trackA, struct Track *trackB,
                         const struct System *system, const struct Timings timings)
{
    struct SpeedInput command;
    struct PWMOutput pwm;
    struct PositionInput positions;
    command.variablePin = ADC_CHANNEL_4;
    command.forwardPin  = GPIO_NUM_25;
    command.backwardPin = GPIO_NUM_26;
    pwm.enable = GPIO_NUM_5;
    pwm.pwm1 = GPIO_NUM_19;
    pwm.pwm2 = GPIO_NUM_18;
    positions.far1 = GPIO_NUM_NC; //GPIO_NUM_35;
    positions.far2 = GPIO_NUM_36;
    positions.close1 = GPIO_NUM_34;
    positions.close2 = GPIO_NUM_39;
    track_new(trackA, "eight track", system, command, pwm, positions, timings);
    command.variablePin = ADC_CHANNEL_5;
    command.forwardPin  = GPIO_NUM_27;
    command.backwardPin = GPIO_NUM_14;
    pwm.enable = GPIO_NUM_4;
    pwm.pwm1 = GPIO_NUM_17;
    pwm.pwm2 = GPIO_NUM_16;
    positions.far1 = GPIO_NUM_NC; //GPIO_NUM_15;
    positions.far2 = GPIO_NUM_23;
    positions.close1 = GPIO_NUM_21;
    positions.close2 = GPIO_NUM_22;
    track_new(trackB, "internal loop", system, command, pwm, positions, timings);
}

void test_set_pin_layout(struct Track *trackA, struct Track *trackB,
                         const struct System *system, const struct Timings timings)
{
    struct SpeedInput command;
    struct PWMOutput pwm;
    struct PositionInput positions;
    command.variablePin = ADC_CHANNEL_4;
    command.forwardPin  = GPIO_NUM_25;
    command.backwardPin = GPIO_NUM_26;
    pwm.enable = GPIO_NUM_5;
    pwm.pwm1 = GPIO_NUM_19;
    pwm.pwm2 = GPIO_NUM_18;
    positions.far1 = GPIO_NUM_NC;
    positions.far2 = GPIO_NUM_NC;
    positions.close1 = GPIO_NUM_NC;
    positions.close2 = GPIO_NUM_NC;
    track_new(trackA, "first track", system, command, pwm, positions, timings);
    command.variablePin = ADC_CHANNEL_5;
    command.forwardPin  = GPIO_NUM_27;
    command.backwardPin = GPIO_NUM_14;
    pwm.enable = GPIO_NUM_4;
    pwm.pwm1 = GPIO_NUM_17;
    pwm.pwm2 = GPIO_NUM_16;
    positions.far1 = GPIO_NUM_NC;
    positions.far2 = GPIO_NUM_NC;
    positions.close1 = GPIO_NUM_NC;
    positions.close2 = GPIO_NUM_NC;
    track_new(trackB, "second track", system, command, pwm, positions, timings);
}

void app_main(void)
{
    /* Debug onboard led */
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 0));

    struct System system;
    system_new(&system);

    struct Timings timings;
    timings.sleepTime = 50;         // Sampling period in milliseconds
    timings.passingSpeed = 2700;    // Max is 4096
    timings.stationSpeed = 1800;    // Idem
    timings.passingDuration = 3500; // Target time for passing
    timings.stationDuration = 4000; // Target time for station stop
    timings.decTarget = 3000;       // Deceleration duration in milliseconds
    timings.accDuration = 4000;     // Acceleration duration in milliseconds
    timings.breakDuration = 300;    // Stopping duration
    timings.stopDuration = 8000;    // Time spent on platform
    timings.stopCount = 3;          // Number of passing in station before a stop
    timings.decDuration = timings.decTarget;

    struct Track trackA, trackB;
    //esp1_set_pin_layout(&trackA, &trackB, &system, timings);
    esp2_set_pin_layout(&trackA, &trackB, &system, timings);
    //test_set_pin_layout(&trackA, &trackB, &system, timings);

    bt_pack_start(&capabilitiesFrame, CAPABILITIES);
    bt_pack_int(&capabilitiesFrame, 2);
    track_setup_capabilities(&trackA);
    track_setup_capabilities(&trackB);

    system_start(&system);
    while (1) {
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 0));
        int trackAUpdated = track_update(&trackA);
        int trackBUpdated = track_update(&trackB);
        if (trackAUpdated || trackBUpdated) {
            ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 1));
        }
        uint64_t count;
        if (system.pingQueue && xQueueReceive(system.pingQueue, &count, 0)) {
            bt_send_ping_frame(count);
        }
        vTaskDelay(pdMS_TO_TICKS(timings.sleepTime));
    }

    track_free(&trackA);
    track_free(&trackB);
    system_free(&system);
}
