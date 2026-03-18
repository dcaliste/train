#include "io.h"

#include "system.h"

#include <esp_log.h>

void speed_input_setup(const struct SpeedInput *command, const struct System *system)
{
    adc_oneshot_chan_cfg_t config =
        {
         .bitwidth = ADC_BITWIDTH_9,
         .atten = ADC_ATTEN_DB_12,
        };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(system->adc, command->variablePin, &config));

    ESP_ERROR_CHECK(gpio_set_direction(command->forwardPin, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(command->forwardPin, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_pullup_en(command->forwardPin));
    ESP_ERROR_CHECK(gpio_set_direction(command->backwardPin, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(command->backwardPin, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_pullup_en(command->backwardPin));
}

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

void command_init(struct Command *command, const struct System *system, struct SpeedInput speed)
{
    command->source = HARDWARE;
    command->hd = speed;
    speed_input_setup(&command->hd, system);
    command->bt.handle = SPP_NO_CLIENT;
}

void command_update(struct Command *command, const struct System *system)
{
    switch (command->source) {
    case (HARDWARE):
        ESP_ERROR_CHECK(adc_oneshot_read(system->adc, command->hd.variablePin, &command->speed));
        command->speed = command->speed ? 512 + 7 * command->speed : 0;
        command->isForward = !gpio_get_level(command->hd.forwardPin);
        command->isBackward = !gpio_get_level(command->hd.backwardPin);
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

uint32_t command_is_bluetooth_controlled(const struct Command *set)
{
    return (set && (set->source == BLUETOOTH)) ? set->bt.handle : SPP_NO_CLIENT;
}

void command_set_bluetooth_controlled(struct Command *command, uint32_t handle)
{
    if (handle == SPP_NO_CLIENT) {
        command->source = HARDWARE;
    } else {
        command->source = BLUETOOTH;
        command->bt.handle = handle;
        command->bt.speed = command->isForward ? command->speed : -command->speed;
    }
}

void command_set_bluetooth_speed(struct Command *set, int speed)
{
    set->bt.speed = speed;
}
