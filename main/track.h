#ifndef TRACK_H
#define TRACK_H

#include "system.h"
#include "io.h"

struct Timings {
    int sleepTime;

    int passingSpeed, stationSpeed;
    int passingDuration, stationDuration;
    int decDuration;
    int decTarget, accDuration, breakDuration;

    int stopDuration, stopCount;
};

enum States {
             SOMEWHERE,
             APPROACHING,
             PASSING_BY,
             STOPPING,
             IN_STATION,
             LEAVING
};

struct Track {
    int id;
    const char* label;

    const struct System *system;
    struct Command command;
    struct PWMOutput pwm;
    struct PositionInput positions;
    struct Timings timings;

    int speed;
    int duration, elapsed;
    int count;
    enum States state;

    struct BtSource stateFrame;
};
void track_new(struct Track *track, const char *label,
               const struct System *system,
               const struct SpeedInput input,
               const struct PWMOutput pwm,
               const struct PositionInput positions,
               const struct Timings timings);
void track_free(struct Track *track);
void track_setup_capabilities(struct Track *track);
int track_update(struct Track *track);

void track_set_dec_duration(struct Track *track, int duration);
void track_set_passing_duration(struct Track *track, int duration);
void track_set_station_duration(struct Track *track, int duration);
void track_set_break_duration(struct Track *track, int duration);
void track_set_stop_duration(struct Track *track, int duration);
void track_set_acc_duration(struct Track *track, int duration);

#endif
