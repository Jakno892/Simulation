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
#define MAX_RADIUS 1.0f
#define ANGULAR_STEP 0.1f
#define RADIAL_STEP 0.02f
#define RADIAL_VELOCITY 0.001f
#define ANGULAR_VELOCITY 0.000002f
#define ALTITUDE STEPS 0.02f
#define MAX_ALTITUDE_CHANGE 1.0f


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

void spiralSearch(cf_state_t* current_state, cf_state_t* desired_state, coord_t* spiralCenter);
void controller(control_t* controlCommands, motorPower_t* motorCommands);
float pid(controlError_t* error, float dt);
// void positionToAttitude(cf_state_t* desiredStatex, cf_state_t* measuredState);
float calc_distance(coord_t point_1, coord_t point_2);
void velocityTransform(cf_state_t* state);
void transform(cf_state_t* state);
void inverse_transform(cf_state_t* state);
coord_t get_velocity(const unsigned char* image);
float constrain(float value, const float minVal, const float maxVal);
#endif