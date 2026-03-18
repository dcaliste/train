#ifndef SYSTEM_H
#define SYSTEM_H

#include <esp_adc/adc_oneshot.h>
#include <driver/mcpwm_timer.h>
#include <driver/mcpwm_oper.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "io.h"

struct System {
    adc_oneshot_unit_handle_t adc;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t pwm;
    QueueHandle_t pingQueue;
};

void system_new(struct System *system);
void system_free(struct System *system);
void system_start(struct System *system);

#define MAX_SPP_SOURCE 256
#define SPP_NO_CLIENT 65535
struct BtSource {
    int len;
    uint8_t data[MAX_SPP_SOURCE];
};
int bt_source_equals(struct BtSource *a, struct BtSource *b);
void bt_source_send_all(struct BtSource *source);
void bt_send_ping_frame(uint64_t count);

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

int bt_pack_start(struct BtSource *at, enum FrameType type);
int bt_pack_bytes(struct BtSource *at, uint8_t *data, int len);
int bt_pack_int(struct BtSource *at, int value);
int bt_pack_int16(struct BtSource *at, uint16_t value);
int bt_pack_str(struct BtSource *at, const char *str);

struct BtSource* capabilitiesFrame();

void bt_register_command(int trackId, struct Command *command);

#endif
