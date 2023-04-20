#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

struct System {
    adc_oneshot_unit_handle_t adc;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t pwm;
};

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
};

void system_free(struct System *system)
{
    ESP_ERROR_CHECK(adc_oneshot_del_unit(system->adc));
    ESP_ERROR_CHECK(mcpwm_del_operator(system->pwm));
    ESP_ERROR_CHECK(mcpwm_del_timer(system->timer));
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

void position_input_setup(const struct PositionInput *positions)
{
    ESP_ERROR_CHECK(gpio_set_direction(positions->close1, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(positions->close2, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(positions->far1, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(positions->far2, GPIO_MODE_INPUT));
}

enum States {
             SOMEWHERE,
             APPROACHING,
             PASSING_BY,
             IN_STATION,
             LEAVING
};

struct Track {
    const struct System *system;
    struct SpeedInput command;
    struct PWMOutput pwm;
    struct PositionInput positions;

    int isForward, isBackward;
    int speed;
    int duration;
    int count;
    enum States state;
};

void track_new(struct Track *track, const struct System *system,
               const struct SpeedInput command,
               const struct PWMOutput pwm,
               const struct PositionInput positions)
{
    track->system = system;

    track->command = command;
    speed_input_setup(&track->command, system);

    track->pwm = pwm;
    pwm_output_setup(&track->pwm, system);

    track->positions = positions;
    position_input_setup(&track->positions);

    track->isForward = 0;
    track->isBackward = 0;
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

#define SLEEP_TIME 50      // Sampling period in milliseconds
#define PASSING_SPEED 3200 // Max is 4096  
#define STATION_SPEED 1856 // Idem
#define DEC_DURATION 3000  // Deceleration duration in milliseconds
#define ACC_DURATION 3000  // Acceleration duration in milliseconds
#define STOP_DURATION 5000 // Deceleration duration in milliseconds
#define STOP_COUNT 3       // Number of passing in station before a stop
int track_update(struct Track *track)
{
    int value;
    ESP_ERROR_CHECK(adc_oneshot_read(track->system->adc, track->command.variablePin, &value));
    value = 512 + 7 * value;

    track->isForward = (value && !gpio_get_level(track->command.forwardPin));
    track->isBackward = (value && !gpio_get_level(track->command.backwardPin));

    if (track->state == SOMEWHERE && ((!gpio_get_level(track->positions.far1) && track->isForward) ||
                                      (!gpio_get_level(track->positions.far2) && track->isBackward))) {
        track->state = APPROACHING;
        track->duration = DEC_DURATION;
        ESP_LOGI("Position", "train approaching");
    } else if (track->state == APPROACHING && ((!gpio_get_level(track->positions.close1) && track->isForward) ||
                                               (!gpio_get_level(track->positions.close2) && track->isBackward))) {
        track->state = PASSING_BY;
        track->duration = 3000;
        ESP_LOGI("Position", "train passing by the station (%d)", track->count);
    /* } else if (track->state == PASSING_BY && ((!gpio_get_level(track->positions.close2) && track->isForward) || */
    /*                                           (!gpio_get_level(track->positions.close1) && track->isBackward))) { */
    /*     track->state = track->count % STOP_COUNT ? LEAVING : IN_STATION; */
    /*     track->duration = track->count % STOP_COUNT ? ACC_DURATION : STOP_DURATION; */
    /*     ESP_LOGI("Position", "train %s", track->count % STOP_COUNT ? "leaving" : "in station"); */
    }
    /* } else if (track->state == LEAVING && ((!gpio_get_level(track->positions.far2) && track->isForward) || */
    /*                                        (!gpio_get_level(track->positions.far1) && track->isBackward))) */
    /*     track->state = SOMEWHERE; */

    int speedLimit = track->count % STOP_COUNT ? PASSING_SPEED : STATION_SPEED;
    if (track->state == APPROACHING) {
        if (track->duration > 0) {
            int rate;
            rate = ((value < speedLimit ? value : speedLimit) - track->speed) * SLEEP_TIME / track->duration;
            value = track->speed + rate;
            track->duration -= SLEEP_TIME;
        } else {
            track->state = PASSING_BY;
            track->duration = 3000;
        }
    } else if (track->state == PASSING_BY) {
        if (track->duration > 0) {
            track->duration -= SLEEP_TIME;
        } else {
            track->state = track->count % STOP_COUNT ? LEAVING : IN_STATION;
            track->duration = track->count % STOP_COUNT ? ACC_DURATION : STOP_DURATION;
        }
        value = value < speedLimit ? value : speedLimit;
    } else if (track->state == IN_STATION) {
        if (track->duration > 0) {
            track->duration -= SLEEP_TIME;
        } else {
            track->state = LEAVING;
            track->duration = ACC_DURATION;
        }
        value = 0;
    } else if (track->state == LEAVING) {
        if (track->duration > 0) {
            int rate;
            rate = (value - track->speed) * SLEEP_TIME / track->duration;
            value = track->speed + rate;
            track->duration -= SLEEP_TIME;
        } else {
            track->state = SOMEWHERE;
            track->count += 1;
        }
    }
    
    int isUpdated = 0;
    ESP_ERROR_CHECK(gpio_set_level(track->pwm.pwm1, track->isForward ? 1 : 0));
    ESP_ERROR_CHECK(gpio_set_level(track->pwm.pwm2, track->isBackward ? 1 : 0));
    if (value != track->speed) {
        isUpdated = (value >> 7 != track->speed >> 7);
        track->speed = value;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(track->pwm.compare, value));
    }
    if (isUpdated)
        ESP_LOGI("Power", "ADC(chan %d) raw data: %d", track->command.variablePin, value);
    return isUpdated;
}

void app_main(void)
{
    /* Debug onboard led */
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 0));

    struct System system;
    system_new(&system);

    struct SpeedInput command;
    struct PWMOutput pwm;
    struct PositionInput positions;
    struct Track trackA, trackB;
    command.variablePin = ADC_CHANNEL_4;
    command.forwardPin  = GPIO_NUM_25;
    command.backwardPin = GPIO_NUM_26;
    pwm.enable = GPIO_NUM_5;
    pwm.pwm1 = GPIO_NUM_19;
    pwm.pwm2 = GPIO_NUM_18;
    positions.far1 = GPIO_NUM_35;
    positions.far2 = GPIO_NUM_39;
    positions.close1 = GPIO_NUM_34;
    positions.close2 = GPIO_NUM_36;
    track_new(&trackA, &system, command, pwm, positions);
    command.variablePin = ADC_CHANNEL_5;
    command.forwardPin  = GPIO_NUM_27;
    command.backwardPin = GPIO_NUM_14;
    pwm.enable = GPIO_NUM_4;
    pwm.pwm1 = GPIO_NUM_17;
    pwm.pwm2 = GPIO_NUM_16;
    positions.far1 = GPIO_NUM_22;
    positions.far2 = GPIO_NUM_1;
    positions.close1 = GPIO_NUM_3;
    positions.close2 = GPIO_NUM_21;
    track_new(&trackB, &system, command, pwm, positions);

    system_start(&system);
    while (1) {
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 0));
        if (track_update(&trackA) || track_update(&trackB)) {
            ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 1));
        }
        vTaskDelay(pdMS_TO_TICKS(SLEEP_TIME));
    }

    track_free(&trackA);
    track_free(&trackB);
    system_free(&system);
}
