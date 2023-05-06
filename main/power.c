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

struct Timings {
    int sleepTime;

    int passingSpeed, stationSpeed;
    int passingDuration, stationDuration;
    int decDuration, accDuration, breakDuration;

    int stopDuration, stopCount;
};

int timings_adjust_speed(int currentSpeed, int targetDuration, int realDuration, int maxSpeed)
{
    int outSpeed;

    outSpeed = currentSpeed * realDuration / targetDuration;
    if (outSpeed < 768)
        outSpeed = 768;
    if (outSpeed > maxSpeed)
        outSpeed = maxSpeed;
    return outSpeed;
}

enum States {
             SOMEWHERE,
             APPROACHING,
             PASSING_BY,
             STOPPING,
             IN_STATION,
             LEAVING
};

struct Track {
    const char* label;

    const struct System *system;
    struct SpeedInput command;
    struct PWMOutput pwm;
    struct PositionInput positions;
    struct Timings timings;

    int isForward, isBackward;
    int speed;
    int duration, elapsed;
    int count;
    enum States state;
};

void track_new(struct Track *track, const char *label,
               const struct System *system,
               const struct SpeedInput command,
               const struct PWMOutput pwm,
               const struct PositionInput positions,
               const struct Timings timings)
{
    track->label = label;

    track->system = system;

    track->command = command;
    speed_input_setup(&track->command, system);

    track->pwm = pwm;
    pwm_output_setup(&track->pwm, system);

    track->positions = positions;
    position_input_setup(&track->positions);

    track->timings = timings;

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

int track_adjust_speed(struct Track *track, int target)
{
    if (track->duration > 0) {
        int rate;
        rate = (target - track->speed) * track->timings.sleepTime / track->duration;
        return track->speed + rate < 0 ? 0 : (track->speed + rate > 4095 ? 4095 : track->speed + rate);
    } else {
        return target;
    }
}

#define AUTO_DETECT 0      // No timer, next transition is based on detection
void track_set_state(struct Track *track, enum States state)
{
    track->state = state;
    switch (state) {
    case SOMEWHERE:
        track->duration = AUTO_DETECT;
        break;
    case APPROACHING:
        track->duration = track->timings.decDuration;
        break;
    case PASSING_BY:
        track->duration = AUTO_DETECT;
        track->elapsed = 0;
        break;
    case LEAVING:
        track->duration = track->timings.accDuration;
        break;
    case STOPPING:
        track->duration = track->timings.breakDuration;
        break;
    case IN_STATION:
        track->duration = track->timings.stopDuration;
        break;
    }
}

int track_state_is_done(const struct Track *track)
{
    return (track->duration <= 0);
}

int track_adjust_state_speed(struct Track *track, int value)
{
    int speedLimit = track->count % track->timings.stopCount
        ? track->timings.passingSpeed : track->timings.stationSpeed;
    speedLimit = value < speedLimit ? value : speedLimit;
    switch (track->state) {
    case (APPROACHING):
        return track_adjust_speed(track, speedLimit);
    case (PASSING_BY):
        return speedLimit;
    case (STOPPING):
        return track_adjust_speed(track, 0);
    case (IN_STATION):
        return 0;
    case (LEAVING):
        return track_adjust_speed(track, value);
    default:
        return value;
    }
}

int track_update(struct Track *track)
{
    int value;
    ESP_ERROR_CHECK(adc_oneshot_read(track->system->adc, track->command.variablePin, &value));
    value = value ? 512 + 7 * value : 0;

    track->isForward = (value && !gpio_get_level(track->command.forwardPin));
    track->isBackward = (value && !gpio_get_level(track->command.backwardPin));

    position_input_log(&track->positions);
    if (track->state == SOMEWHERE
        && ((position_input_isTriggered(&track->positions, FAR_1) && track->isForward) ||
            (position_input_isTriggered(&track->positions, FAR_2) && track->isBackward))) {
        track_set_state(track, APPROACHING);
        ESP_LOGI("Position", "%s: train approaching", track->label);
    } else if (track->state == APPROACHING
               && ((position_input_isTriggered(&track->positions, CLOSE_1) && track->isForward) ||
                   (position_input_isTriggered(&track->positions, CLOSE_2) && track->isBackward))) {
        track_set_state(track, PASSING_BY);
        ESP_LOGI("Position", "%s: train passing by the station (%d)", track->label, track->count);
    } else if (track->state == PASSING_BY
               && ((position_input_isTriggered(&track->positions, CLOSE_2) && track->isForward) ||
                   (position_input_isTriggered(&track->positions, CLOSE_1) && track->isBackward))) {
        if (track->count % track->timings.stopCount) {
            track_set_state(track, LEAVING);
            track->timings.passingSpeed = timings_adjust_speed(track->timings.passingSpeed,
                                                               track->timings.passingDuration, track->elapsed, 3072);
            ESP_LOGI("Position", "%s: adjust passing speed %d", track->label, track->timings.passingSpeed);
        } else {
            track_set_state(track, STOPPING);
            track->timings.stationSpeed = timings_adjust_speed(track->timings.stationSpeed,
                                                               track->timings.stationDuration, track->elapsed, 2048);
            ESP_LOGI("Position", "%s: adjust station speed %d", track->label, track->timings.stationSpeed);
        }
        ESP_LOGI("Position", "%s: train at platform end (in %d ms)", track->label, track->elapsed);
    } else if (track->state == STOPPING && !track->speed) {
        track_set_state(track, IN_STATION);
        ESP_LOGI("Position", "%s: train is stopped", track->label);
    } else if (track->state == IN_STATION && track_state_is_done(track)) {
        track_set_state(track, LEAVING);
        ESP_LOGI("Position", "%s: train is departing", track->label);
    } else if (track->state == LEAVING && track->speed >= value) {
        track_set_state(track, SOMEWHERE);
        track->count += 1;
        ESP_LOGI("Position", "%s: train has done %d passings", track->label, track->count);
    }

    value = track_adjust_state_speed(track, value);

    if (track->duration > 0 && (track->isBackward || track->isForward))
        track->duration -= track->timings.sleepTime;
    if (track->state == PASSING_BY && (track->isBackward || track->isForward))
        track->elapsed += track->timings.sleepTime;
    
    int isUpdated = 0;
    ESP_ERROR_CHECK(gpio_set_level(track->pwm.pwm1, track->isForward ? 1 : 0));
    ESP_ERROR_CHECK(gpio_set_level(track->pwm.pwm2, track->isBackward ? 1 : 0));
    if (value != track->speed) {
        isUpdated = (value >> 7 != track->speed >> 7);
        track->speed = value;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(track->pwm.compare, value));
    }
    if (isUpdated) {
        ESP_LOGI("Power", "%s: PWM updated value: %d", track->label, value);
    }
    return isUpdated;
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
    timings.passingDuration = 2000; // Target time for passing
    timings.stationDuration = 3000; // Target time for station stop
    timings.decDuration = 3000;     // Deceleration duration in milliseconds
    timings.accDuration = 3000;     // Acceleration duration in milliseconds
    timings.breakDuration = 300;    // Stopping duration
    timings.stopDuration = 8000;    // Time spent on platform
    timings.stopCount = 3;          // Number of passing in station before a stop

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
    positions.far2 = GPIO_NUM_NC; //GPIO_NUM_36;
    positions.close1 = GPIO_NUM_34;
    positions.close2 = GPIO_NUM_39;
    track_new(&trackA, "track A", &system, command, pwm, positions, timings);
    command.variablePin = ADC_CHANNEL_5;
    command.forwardPin  = GPIO_NUM_27;
    command.backwardPin = GPIO_NUM_14;
    pwm.enable = GPIO_NUM_4;
    pwm.pwm1 = GPIO_NUM_17;
    pwm.pwm2 = GPIO_NUM_16;
    positions.far1 = GPIO_NUM_NC; //GPIO_NUM_22;
    positions.far2 = GPIO_NUM_NC; //GPIO_NUM_1;
    positions.close1 = GPIO_NUM_NC; //GPIO_NUM_3;
    positions.close2 = GPIO_NUM_NC; //GPIO_NUM_21;
    track_new(&trackB, "track B", &system, command, pwm, positions, timings);

    system_start(&system);
    while (1) {
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 0));
        int trackAUpdated = track_update(&trackA);
        int trackBUpdated = track_update(&trackB);
        if (trackAUpdated || trackBUpdated) {
            ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_2, 1));
        }
        vTaskDelay(pdMS_TO_TICKS(timings.sleepTime));
    }

    track_free(&trackA);
    track_free(&trackB);
    system_free(&system);
}
