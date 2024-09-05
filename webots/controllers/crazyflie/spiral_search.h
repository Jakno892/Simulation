#ifndef SPIRAL_SEARCH_H_
#define SPIRAL_SEARCH_H_
#include "pid_controller.h"
// #include "path_planner.h"
#include <stdlib.h>

typedef struct cf_state
{
    float x, y, z;
    float vx, vy, vz;
    float roll, pitch, yaw;
    float yaw_rate;
    float altitude;
    float dt;
    float r;
    float r_max;
} cf_state_t;

typedef struct coordinate
{
    float x, y, z;
    float yaw;
} coord_t;

void spiral_search(cf_state_t* state, float r_step, float yaw_rate);
void engage(cf_state_t* desiredState, float path);
void pid_controller(cf_state_t* measured_state, cf_state_t* desired_state, gainsPid_t gains_pid, float dt, motorPower_t* motorCommands);
float calc_distance(coord_t point_1, coord_t point_2);
#endif