#include "track.h"

#include <esp_log.h>

#define ABS(a) ((a) < 0 ? -(a) : (a))
#define MAX(a, b) ((a) < (b) ? (b) : (a))

static void sort_ab(const int *durations, int target, int *a, int *b)
{
    if (ABS(target - durations[*a]) > ABS(target - durations[*b])) {
        int tmp = *a;
        *a = *b;
        *b = tmp;
    }
}

static int timings_adjust_speed(const int *durations, const int *speeds, int count,
                                int targetDuration, int minSpeed, int maxSpeed)
{
    int speed;
    if (!count)
        speed = speeds[0] * durations[0] / targetDuration;
    else {
        /* Find the closest history points to the target, and compare with previous. */
        int a = count % HISTORY_LENGTH, b = (count - 1) % HISTORY_LENGTH;
        for (int i = count - 2; i >= 0 && i > count - HISTORY_LENGTH; i--) {
            if (ABS(targetDuration - durations[i % HISTORY_LENGTH]) < ABS(targetDuration - durations[b])
                && durations[i % HISTORY_LENGTH] != durations[a])
                b = i % HISTORY_LENGTH;
        }
        sort_ab(durations, targetDuration, &a, &b);
        if (durations[a] != durations[b])
            speed = speeds[a] + (speeds[b] - speeds[a]) * (targetDuration - durations[a]) / (durations[b] - durations[a]);
        else
            speed = speeds[a];
    }
    if (speed < minSpeed)
        speed = minSpeed;
    if (speed > maxSpeed)
        speed = maxSpeed;
    return speed;
}

static void timings_adjust_duration(int *duration, int targetDuration, int realDuration)
{
    *duration = *duration * realDuration / targetDuration;
    if (*duration < 100)
        *duration = 100;
    if (*duration > 8000)
        *duration = 8000;
}

static void timings_store_history(int *history, int count, int duration)
{
    history[count % HISTORY_LENGTH] = duration;
}

void track_new(struct Track *track, const char *label,
               const struct System *system,
               const struct SpeedInput input,
               const struct PWMOutput pwm,
               const struct PositionInput positions,
               const struct Timings timings)
{
    static int id = 0;
    uint16_t duration;

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

    ESP_ERROR_CHECK(nvs_open(track->label, NVS_READWRITE, &track->nvs));
    if (nvs_get_u16(track->nvs, "decDuration", &duration) == ESP_OK) {
        track->timings.decTarget = duration;
    }
    if (nvs_get_u16(track->nvs, "passingDuration", &duration) == ESP_OK) {
        track->timings.passingDuration = duration;
    }
    if (nvs_get_u16(track->nvs, "stationDuration", &duration) == ESP_OK) {
        track->timings.stationDuration = duration;
    }
    if (nvs_get_u16(track->nvs, "breakDuration", &duration) == ESP_OK) {
        track->timings.breakDuration = duration;
    }
    if (nvs_get_u16(track->nvs, "stopDuration", &duration) == ESP_OK) {
        track->timings.stopDuration = duration;
    }
    if (nvs_get_u16(track->nvs, "accDuration", &duration) == ESP_OK) {
        track->timings.accDuration = duration;
    }
}

void track_free(struct Track *track)
{
    ESP_ERROR_CHECK(mcpwm_del_generator(track->pwm.generate));
    ESP_ERROR_CHECK(mcpwm_del_comparator(track->pwm.compare));
    nvs_close(track->nvs);
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
            timings_store_history(track->timings.decHistory, track->count, track->elapsed);
            timings_adjust_duration(&track->timings.decDuration,
                                    track->timings.decTarget, track->elapsed);
        }
        track->duration = AUTO_DETECT;
        track->elapsed = 0;
        break;
    case LEAVING:
        if (track->state == PASSING_BY) {
            timings_store_history(track->timings.passingHistory, track->count, track->elapsed);
            timings_store_history(track->timings.passingSpeedHistory, track->count, track->timings.passingSpeed);
            track->timings.passingSpeed =
                timings_adjust_speed(track->timings.passingHistory,
                                     track->timings.passingSpeedHistory, track->count,
                                     track->timings.passingDuration,
                                     track->timings.minSpeed, (MAX_SPEED >> 1) + (MAX_SPEED >> 2));
        }
        track->duration = track->timings.accDuration;
        track->elapsed = -1;
        break;
    case STOPPING:
        if (track->state == PASSING_BY) {
            timings_store_history(track->timings.passingHistory, track->count, track->elapsed);
            timings_store_history(track->timings.passingSpeedHistory, track->count, track->timings.stationSpeed);
            track->timings.stationSpeed =
                timings_adjust_speed(track->timings.passingHistory,
                                     track->timings.passingSpeedHistory, track->count,
                                     track->timings.stationDuration,
                                     track->timings.minSpeed, MAX_SPEED >> 1);
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
    if (track->timings.decTarget != duration) {
        ESP_LOGI("Timings", "%s: set decDuration %d (was %d)", track->label, duration, track->timings.decTarget);
        track->timings.decTarget = duration;
        nvs_set_u16(track->nvs, "decDuration", duration);
        ESP_ERROR_CHECK(nvs_commit(track->nvs));
    }
}

void track_set_passing_duration(struct Track *track, int duration)
{
    if (track->timings.passingDuration != duration) {
        ESP_LOGI("Timings", "%s: set passingDuration %d (was %d)", track->label, duration, track->timings.passingDuration);
        track->timings.passingDuration = duration;
        nvs_set_u16(track->nvs, "passingDuration", duration);
        ESP_ERROR_CHECK(nvs_commit(track->nvs));
    }
}

void track_set_station_duration(struct Track *track, int duration)
{
    if (track->timings.stationDuration != duration) {
        ESP_LOGI("Timings", "%s: set stationDuration %d (was %d)", track->label, duration, track->timings.stationDuration);
        track->timings.stationDuration = duration;
        nvs_set_u16(track->nvs, "stationDuration", duration);
        ESP_ERROR_CHECK(nvs_commit(track->nvs));
    }
}

void track_set_break_duration(struct Track *track, int duration)
{
    if (track->timings.breakDuration != duration) {
        ESP_LOGI("Timings", "%s: set breakDuration %d (was %d)", track->label, duration, track->timings.breakDuration);
        track->timings.breakDuration = duration;
        nvs_set_u16(track->nvs, "breakDuration", duration);
        ESP_ERROR_CHECK(nvs_commit(track->nvs));
    }
}

void track_set_stop_duration(struct Track *track, int duration)
{
    if (track->timings.stopDuration != duration) {
        ESP_LOGI("Timings", "%s: set stopDuration %d (was %d)", track->label, duration, track->timings.stopDuration);
        track->timings.stopDuration = duration;
        nvs_set_u16(track->nvs, "stopDuration", duration);
        ESP_ERROR_CHECK(nvs_commit(track->nvs));
    }
}

void track_set_acc_duration(struct Track *track, int duration)
{
    if (track->timings.accDuration != duration) {
        ESP_LOGI("Timings", "%s: set accDuration %d (was %d)", track->label, duration, track->timings.accDuration);
        track->timings.accDuration = duration;
        nvs_set_u16(track->nvs, "accDuration", duration);
        ESP_ERROR_CHECK(nvs_commit(track->nvs));
    }
}

