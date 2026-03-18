#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <mdns.h>

#include "wifi.h"
#include "system.h"
#include "io.h"
#include "track.h"

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
    positions.far1 = GPIO_NUM_35;
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
    positions.far1 = GPIO_NUM_15;
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

#define TARGET "traintest"
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(TARGET));
    ESP_ERROR_CHECK(mdns_instance_name_set("Train controller"));

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
    if (!strcmp(TARGET, "train1"))
        esp1_set_pin_layout(&trackA, &trackB, &system, timings);
    else if (!strcmp(TARGET, "train2"))
        esp2_set_pin_layout(&trackA, &trackB, &system, timings);
    else
        test_set_pin_layout(&trackA, &trackB, &system, timings);

    bt_pack_start(capabilitiesFrame(), CAPABILITIES);
    bt_pack_int(capabilitiesFrame(), 2);
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
