#ifndef SPIRAL_SEARCH_H_
#define SPIRAL_SEARCH_H_
// #include "pid_controller.h"
// #include "path_planner.h"
#include <stdlib.h>
#define FLYING_ALTITUDE 1.5

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define g 9.81


typedef struct cf_state
{
    float x, y, z; // Global position
    float vx, vy, vz; // Global velocity
    float roll, pitch, yaw;
    float yaw_rate;
    float altitude;
    float dt;
} cf_state_t;

typedef struct coordinate
{
    float x, y, z;
    float yaw;
} coord_t;

typedef struct motor_power_s {
  float frontLeft;
  float rearLeft;
  float rearRight;
  float frontRight;
} motorPower_t;

typedef struct control_commands_s {
  float roll;
  float pitch;
  float yaw;
  float altitude;
} control_t;

typedef struct controlError_s {
  float error;
  float previousError;
  float integrator, intMax;
  float kp, ki, kd;
} controlError_t;
  
typedef struct gainsPid {
    float kp_pos_xy, kp_pos_z;     // Proportional gain for position control
    float ki_pos_xy, ki_pos_z;     // Integral gain for position control
    float kd_pos_xy, kd_pos_z;     // Derivative gain for position control
    float kp_vel_xy, kp_vel_z;     // Proportional gain for velocity control
    float kd_vel_xy, kd_vel_z;     // Derivative gain for velocity control
} gainsPid_t;

void spiral_search(cf_state_t* current_state, cf_state_t* desired_state, float r_step, float yaw_rate);
void controller(control_t* controlCommands, motorPower_t* motorCommands);
float pid(controlError_t* error, float dt);
// void positionToAttitude(cf_state_t* desiredStatex, cf_state_t* measuredState);
float calc_distance(coord_t point_1, coord_t point_2);
coord_t transform(coord_t* rpy, coord_t* xyz);
coord_t inverse_transform(coord_t* rpy, coord_t* xyz);
coord_t get_velocity(const unsigned char* image);
float constrain(float value, const float minVal, const float maxVal);
#endif