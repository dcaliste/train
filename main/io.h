#ifndef IO_H
#define IO_H

#include <esp_adc/adc_oneshot.h>
#include <driver/gpio.h>
#include <driver/mcpwm_cmpr.h>
#include <driver/mcpwm_gen.h>

struct System;

struct SpeedInput {
    adc_channel_t variablePin;
    gpio_num_t forwardPin, backwardPin;
};
void speed_input_setup(const struct SpeedInput *command, const struct System *system);

enum Source {
             HARDWARE,
             BLUETOOTH
};

struct BtCommand {
    uint32_t handle;
    int speed;
};

struct Command
{
    enum Source source;
    struct SpeedInput hd;
    struct BtCommand bt;
    int speed;
    int isForward, isBackward;
};
void command_init(struct Command *command, const struct System *system, struct SpeedInput speed);
void command_update(struct Command *command, const struct System *system);
uint32_t command_is_bluetooth_controlled(const struct Command *set);
void command_set_bluetooth_controlled(struct Command *command, uint32_t handle);
void command_set_bluetooth_speed(struct Command *set, int speed);

enum Positions {
                CLOSE_1,
                CLOSE_2,
                FAR_1,
                FAR_2
};

struct PositionInput {
    gpio_num_t close1, close2;
    gpio_num_t far1, far2;
};
void position_input_setup(const struct PositionInput *positions);
int position_input_isEnabled(const struct PositionInput *positions);
void position_input_log(const struct PositionInput *positions);
int position_input_isTriggered(const struct PositionInput *positions, enum Positions position);

struct PWMOutput {
    gpio_num_t enable, pwm1, pwm2;
    mcpwm_cmpr_handle_t compare;
    mcpwm_gen_handle_t generate;
};
void pwm_output_setup(struct PWMOutput *pwm, const struct System *system);

#endif
