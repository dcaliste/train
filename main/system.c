#include "system.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_sdp_api.h"
#include "esp_spp_api.h"
#include "esp_log.h"
#include <driver/gptimer.h>

gptimer_handle_t pingTimer;

struct BtPayload {
    int len;
    uint8_t *data;
};

#define MAX_SPP_CLIENTS 5
struct SppClient {
    uint32_t handle;
    struct BtPayload payload;
    int alive;
};
static struct SppClient sppClients[MAX_SPP_CLIENTS];

static struct BtPayload* bt_payload(uint32_t handle)
{
    for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
        if (sppClients[i].handle == handle) {
            return &sppClients[i].payload;
        }
    }
    ESP_LOGI("Bluetooth", "unknown handle: %ld", handle);
    return 0;
}

static struct BtSource capabilitiesFrame_;
struct BtSource* capabilitiesFrame()
{
    return &capabilitiesFrame_;
}

#define MAX_COMMANDS 2
static struct Command *commands[MAX_COMMANDS];

void bt_register_command(int trackId, struct Command *command)
{
    if (trackId < 0 || trackId >= MAX_COMMANDS)
        return;

    commands[trackId] = command;
    ESP_LOGI("Bluetooth", "registering track %d: %p", trackId, commands[trackId]);
}

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

static void bt_spp_send(struct SppClient *spp, struct BtSource *source)
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

static void bt_source_send_new(uint32_t handle, struct BtSource *source)
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

int bt_pack_bytes(struct BtSource *at, uint8_t *data, int len)
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
    return bt_pack_bytes(at, (uint8_t*)&type, sizeof(enum FrameType) / sizeof(uint8_t));
}
int bt_pack_int(struct BtSource *at, int value)
{
    return bt_pack_bytes(at, (uint8_t*)&value, sizeof(int) / sizeof(uint8_t));
}
int bt_pack_int16(struct BtSource *at, uint16_t value)
{
    return bt_pack_bytes(at, (uint8_t*)&value, sizeof(uint16_t) / sizeof(uint8_t));
}
int bt_pack_uint64(struct BtSource *at, uint64_t value)
{
    return bt_pack_bytes(at, (uint8_t*)&value, sizeof(uint64_t) / sizeof(uint8_t));
}
int bt_pack_str(struct BtSource *at, const char *str)
{
    return bt_pack_bytes(at, (uint8_t*)str, (strlen(str) + 1) * sizeof(char) / sizeof(uint8_t));
}

static uint8_t* bt_get_frame_data(struct BtPayload *payload,
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

static void bt_get_frame_command(uint8_t **frame, int *trackId, struct Command **command)
{
    if (command && trackId) {
        *trackId = *(int*)*frame;
        if (*trackId >= 0 && *trackId < MAX_COMMANDS)
            *command = commands[*trackId];
        else
            *command = (struct Command*)0;
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

static void bt_receive_ping_frame(uint32_t handle, uint64_t count)
{
    ESP_LOGI("Bluetooth", "ping response: %lld (%ld)", count, handle);
    for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
        if (sppClients[i].handle == handle && !sppClients[i].alive) {
            sppClients[i].alive = 1;
        }
    }
}

static int isPingFrame(struct BtPayload *payload, uint64_t *count)
{
    static int pingLen = sizeof(enum FrameType) + sizeof(uint64_t);

    const uint8_t *frame = bt_get_frame_data(payload, PING, pingLen);
    if (frame && count) {
        *count = *(uint64_t*)frame;
    }
    return (frame != (uint8_t*)0);
}

static void bt_receive_acquire_frame(uint32_t handle, int trackId, struct Command *command)
{
    int acquire = command && (command_is_bluetooth_controlled(command) == SPP_NO_CLIENT);
    struct BtSource ack;
    bt_pack_start(&ack, ACQUIRE_ACK);
    bt_pack_int(&ack, trackId);
    bt_pack_int(&ack, acquire);
    ESP_LOGI("Bluetooth", "acquire response for track %d: %d", trackId, acquire);
    if (command && acquire)
        command_set_bluetooth_controlled(command, handle);

    for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
        if (sppClients[i].handle == handle) {
            bt_spp_send(sppClients + i, &ack);
            return;
        }
    }
}

static int isAcquireFrame(struct BtPayload *payload, int *trackId, struct Command **command)
{
    static int acquireLen = sizeof(enum FrameType) + sizeof(uint32_t);

    uint8_t *frame = bt_get_frame_data(payload, ACQUIRE_TRACK, acquireLen);
    if (frame) {
        bt_get_frame_command(&frame, trackId, command);
    }
    return (frame != (uint8_t*)0);
}

static void bt_receive_release_frame(uint32_t handle, int trackId, struct Command *command)
{
    int release = command && (command_is_bluetooth_controlled(command) == handle);
    struct BtSource ack;
    bt_pack_start(&ack, RELEASE_ACK);
    bt_pack_int(&ack, trackId);
    bt_pack_int(&ack, release);
    ESP_LOGI("Bluetooth", "release response for track %d: %d", trackId, release);
    if (command && release)
        command_set_bluetooth_controlled(command, SPP_NO_CLIENT);

    for (int i = 0; i < MAX_SPP_CLIENTS; i++) {
        if (sppClients[i].handle == handle) {
            bt_spp_send(sppClients + i, &ack);
            return;
        }
    }
}

static int isReleaseFrame(struct BtPayload *payload, int *trackId, struct Command **command)
{
    static int releaseLen = sizeof(enum FrameType) + sizeof(uint32_t);

    uint8_t *frame = bt_get_frame_data(payload, RELEASE_TRACK, releaseLen);
    if (frame) {
        bt_get_frame_command(&frame, trackId, command);
    }
    return (frame != (uint8_t*)0);
}

static int isSpeedFrame(struct BtPayload *payload,
                        int *trackId, struct Command **command, int *speed)
{
    static int speedLen = sizeof(enum FrameType) + sizeof(uint32_t) + sizeof(int32_t);

    uint8_t *frame = bt_get_frame_data(payload, SPEED_COMMAND, speedLen);
    if (frame) {
        bt_get_frame_command(&frame, trackId, command);
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
        for (int i = 0; i < MAX_COMMANDS; i++) {
            if (commands[i] && (command_is_bluetooth_controlled(commands[i]) == param->close.handle)) {
                command_set_bluetooth_controlled(commands[i], SPP_NO_CLIENT);
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
        bt_source_send_new(param->srv_open.handle, &capabilitiesFrame_);
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
            struct Command *command;
            struct BtPayload payload;
            payload.data = param->data_ind.data;
            payload.len = param->data_ind.len;
            while (payload.len > 0) {
                if (isPingFrame(&payload, &ping)) {
                    bt_receive_ping_frame(param->data_ind.handle, ping);
                } else if (isAcquireFrame(&payload, &trackId, &command)) {
                    bt_receive_acquire_frame(param->data_ind.handle, trackId, command);
                } else if (isReleaseFrame(&payload, &trackId, &command)) {
                    bt_receive_release_frame(param->data_ind.handle, trackId, command);
                } else if (isSpeedFrame(&payload, &trackId, &command, &speed) && command) {
                    command_set_bluetooth_speed(command, speed);
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
            esp_bt_gap_set_device_name("ESP train");
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

    for (int i = 0; i < MAX_COMMANDS; i++)
        commands[i] = (struct Command*)0;

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
}

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
