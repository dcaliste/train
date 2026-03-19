#include "track.h"

#include <esp_log.h>

static void timings_adjust_speed(int *speed, int targetDuration, int realDuration, int maxSpeed)
{
    *speed = *speed * realDuration / targetDuration;
    if (*speed < 768)
        *speed = 768;
    if (*speed > maxSpeed)
        *speed = maxSpeed;
}

static void timings_adjust_duration(int *duration, int targetDuration, int realDuration)
{
    *duration = *duration * realDuration / targetDuration;
    if (*duration < 300)
        *duration = 300;
    if (*duration > 6000)
        *duration = 6000;
}

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

    command_init(&track->command, system, input);
    bt_register_command(track->id, &track->command);

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

static int bt_pack_track_state(struct BtSource *at, enum States state)
{
    return bt_pack_bytes(at, (uint8_t*)&state, sizeof(enum States) / sizeof(uint8_t));
}

enum Capabilities {
                   SPEED_CONTROL = 1,
                   POSITIONING   = 2
};

#define MAX_SPEED 4095     // Maximum speed value

void track_setup_capabilities(struct Track *track)
{
    bt_pack_int(capabilitiesFrame(), track->id);
    bt_pack_int(capabilitiesFrame(), strlen(track->label));
    bt_pack_str(capabilitiesFrame(), track->label);
    bt_pack_int(capabilitiesFrame(), MAX_SPEED);
    uint16_t flag = SPEED_CONTROL;
    if (position_input_isEnabled(&track->positions)) {
        flag |= POSITIONING;
    }
    bt_pack_int16(capabilitiesFrame(), flag);
}

#define AUTO_DETECT 0      // No timer, next transition is based on detection
static void track_set_state(struct Track *track, enum States state)
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

static int track_state_is_done(const struct Track *track)
{
    return (track->duration <= 0);
}

static int track_state_get_speed_target(struct Track *track)
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

static int track_state_adjust_speed(struct Track *track, int value)
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

static int track_update_frame(struct Track *track)
{
    struct BtSource old = track->stateFrame;

    bt_pack_start(&track->stateFrame, TRACK_STATE);
    bt_pack_int(&track->stateFrame, track->id);
    bt_pack_int(&track->stateFrame, track->command.isForward);
    bt_pack_int(&track->stateFrame, track->command.isBackward);
    bt_pack_int(&track->stateFrame, track->speed);
    bt_pack_int(&track->stateFrame, track->count);
    bt_pack_track_state(&track->stateFrame, track->state);

    return !bt_source_equals(&old, &track->stateFrame);
}

int track_update(struct Track *track)
{
    command_update(&track->command, track->system);

    position_input_log(&track->positions);
    if (track->state == SOMEWHERE
        && ((position_input_isTriggered(&track->positions, FAR_1) && track->command.isForward) ||
            (position_input_isTriggered(&track->positions, FAR_2) && track->command.isBackward))) {
        track_set_state(track, APPROACHING);
        ESP_LOGI("Position", "%s: train approaching", track->label);
    } else if (track->state == APPROACHING
               && ((position_input_isTriggered(&track->positions, CLOSE_1) && track->command.isForward) ||
                   (position_input_isTriggered(&track->positions, CLOSE_2) && track->command.isBackward))) {
        track_set_state(track, PASSING_BY);
        ESP_LOGI("Position", "%s: train breaks for the station in %d ms", track->label, track->elapsed);
        ESP_LOGI("Position", "%s: adjust approaching duration %d", track->label, track->timings.decDuration);
        ESP_LOGI("Position", "%s: train passing by the station (%d)", track->label, track->count);
    } else if (track->state == PASSING_BY
               && ((position_input_isTriggered(&track->positions, CLOSE_2) && track->command.isForward) ||
                   (position_input_isTriggered(&track->positions, CLOSE_1) && track->command.isBackward))) {
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
    } else if (track->state == LEAVING && track->speed >= track->command.speed) {
        track_set_state(track, SOMEWHERE);
        track->count += 1;
        ESP_LOGI("Position", "%s: train has done %d passings", track->label, track->count);
    }

    if (track->duration > 0 && (track->command.isBackward || track->command.isForward))
        track->duration -= track->timings.sleepTime;
    if (track->elapsed >= 0 && (track->command.isBackward || track->command.isForward))
        track->elapsed += track->timings.sleepTime;
    
    int isUpdated = 0;
    ESP_ERROR_CHECK(gpio_set_level(track->pwm.pwm1, track->command.isForward ? 1 : 0));
    ESP_ERROR_CHECK(gpio_set_level(track->pwm.pwm2, track->command.isBackward ? 1 : 0));

    int value = track_state_adjust_speed(track, track->command.speed);
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

void track_set_dec_duration(struct Track *track, int duration)
{
    if (track->timings.decDuration != duration) {
        ESP_LOGI("Timings", "%s: set decDuration %d (was %d)", track->label, duration, track->timings.decDuration);
        track->timings.decDuration = duration;
    }
}

void track_set_passing_duration(struct Track *track, int duration)
{
    if (track->timings.passingDuration != duration) {
        ESP_LOGI("Timings", "%s: set passingDuration %d (was %d)", track->label, duration, track->timings.passingDuration);
        track->timings.passingDuration = duration;
    }
}

void track_set_station_duration(struct Track *track, int duration)
{
    if (track->timings.stationDuration != duration) {
        ESP_LOGI("Timings", "%s: set stationDuration %d (was %d)", track->label, duration, track->timings.stationDuration);
        track->timings.stationDuration = duration;
    }
}

void track_set_break_duration(struct Track *track, int duration)
{
    if (track->timings.breakDuration != duration) {
        ESP_LOGI("Timings", "%s: set breakDuration %d (was %d)", track->label, duration, track->timings.breakDuration);
        track->timings.breakDuration = duration;
    }
}

void track_set_stop_duration(struct Track *track, int duration)
{
    if (track->timings.stopDuration != duration) {
        ESP_LOGI("Timings", "%s: set stopDuration %d (was %d)", track->label, duration, track->timings.stopDuration);
        track->timings.stopDuration = duration;
    }
}

void track_set_acc_duration(struct Track *track, int duration)
{
    if (track->timings.accDuration != duration) {
        ESP_LOGI("Timings", "%s: set accDuration %d (was %d)", track->label, duration, track->timings.accDuration);
        track->timings.accDuration = duration;
    }
}

