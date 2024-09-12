#include "spiral_search.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

void spiral_search(cf_state_t *measuredState, cf_state_t *desiredState, float r_step, float yaw_rate)
{
  float angle = measuredState->yaw;
  float radius = 1;
  float r_max = 5;

  // Increase the spiral radius and yaw angle.
  printf("actual radius: %f \n",radius);

  if (radius < r_max)
  {
    radius += r_step;
  }
  angle += yaw_rate;

  // Set the desired x and y speeds
  measuredState->vx = radius;

  // Update the relative position
  measuredState->x = measuredState->vx;
  measuredState->y = measuredState->vy;

  // Update the radius and yaw

  // measuredState->r = radius;
  measuredState->yaw = angle;
}

float pid(controlError_t* error, float dt)
{  
  float proportional = error->kp * constrain(error->error, -1, 1);  // Only constrain error here
  float integral = error->ki * error->integrator;
  float derivative = error->kd * (error->error - error->previousError) / dt;
  
  float controlSignal = proportional + integral + derivative;
  
  error->integrator += error->error * dt;
  // error->integrator = constrain(error->integrator, -error->intMax, error->intMax);  // Constrain to avoid windup
  
  error->previousError = error->error;
  
  return controlSignal;
}

void controller(control_t* controlCommands, motorPower_t* motorCommands)
{
  motorCommands->frontLeft = constrain(controlCommands->altitude - controlCommands->roll + controlCommands->pitch + controlCommands->yaw, -600, 600);
  motorCommands->rearLeft = constrain(controlCommands->altitude - controlCommands->roll - controlCommands->pitch - controlCommands->yaw, -600, 600);
  motorCommands->rearRight = constrain(controlCommands->altitude + controlCommands->roll - controlCommands->pitch + controlCommands->yaw, -600, 600);
  motorCommands->frontRight = constrain(controlCommands->altitude + controlCommands->roll + controlCommands->pitch - controlCommands->yaw, -600, 600);
}

// void positionToAttitude(cf_state_t* desiredState, cf_state_t* measuredState)
// {

//   controlError_t x_velocityError = 
//   {
//     .error = desiredState->x - measuredState->x,
//     .kp = 2,
//     .kd = 0.5
//   };
//   // Calculate horizontal position errors
//   float dx = desiredState->x - measuredState->x;
//   float dy = desiredState->y - measuredState->y;

//   // Calculate desired horizontal velocities (e.g., proportionally slowing as you near target)
//   desiredState->vx = dx * 1;
//   desiredState->vy = dy * 1;

//   // Calculate roll and pitch angles from velocity
//   desiredState->roll = atan2(desiredState->vy, g);
//   desiredState->pitch = atan2(desiredState->vx, g);

// }

float constrain(float value, const float minVal, const float maxVal)
{
  return fminf(maxVal, fmaxf(minVal, value));
}

float calc_distance(coord_t point_1, coord_t point_2)
{
  return sqrt(pow(point_1.x - point_2.x,2) + pow(point_1.y - point_2.y,2) + pow(point_1.z - point_2.z,2));
}

coord_t transform(coord_t *rpy, coord_t *xyz)
{
  float sr = sin(rpy->x);
  float sp = sin(rpy->y); // Sine values of roll, pitch and yaw
  float sy = sin(rpy->z);

  float cr = cos(rpy->x);
  float cp = cos(rpy->y); // Cosine values of roll, pitch and yaw
  float cy = cos(rpy->z);

  coord_t global = {0};

  // Rotate the coordinate system from bidy fixed coordinates to global coordinates

  global.x = cy*cp*xyz->x +
            (cy*sp*sr - sy*cr)*xyz->y + 
            (cy*sp*cr + sy*sr)*xyz->z;
  
  global.y = sy*cp*xyz->x + 
            (sy*sp*sr + cy*cr)*xyz->y + 
            (sy*sp*cr - cy*sr)*xyz->z;

  global.z = -sp*xyz->x + cp*sr*xyz->y + cp*cr*xyz->z;

  return global;
}

coord_t inverse_transform(coord_t *rpy, coord_t *xyz)
{
  float sr = sin(rpy->x);
  float sp = sin(rpy->y); // Sine values of roll, pitch and yaw
  float sy = sin(rpy->z);

  float cr = cos(rpy->x);
  float cp = cos(rpy->y); // Cosine values of roll, pitch and yaw
  float cy = cos(rpy->z);

  coord_t body_fixed = {0};

  // Rotate the coordinate system from global coordinates to body fixed coordinates

  body_fixed.x = cy*cp*xyz->x + sy*cp*xyz->y + -sp*xyz->z;
               
  body_fixed.y = (cy*sp*sr - sy*cr)*xyz->x + (sy*sp*sr + cy*cr)*xyz->y + cp*sr*xyz->z;

  body_fixed.z = (cy*sp*cr + sy*sr)*xyz->x + (sy*sp*cr - cy*sr)*xyz->y + cp*cr*xyz->z;

  return body_fixed;
}

coord_t get_velocity(const unsigned char *image)
{
  coord_t x = {0};
  return x;
}

// Function to move to the next waypoint
void move_to_next_waypoint(int index, int max)
{
    if (index < max - 1) {
        index++;
    }
}

// float constrain(float value, const float minVal, const float maxVal) {
//   return fminf(maxVal, fmaxf(minVal, value));
// }

void spiral_search_2d(cf_state_t *measuredState, cf_state_t *desiredState) {
    static float r = 0.05;  // Start radius of the spiral
    static float angle = 0.0;  // Angular position in radians
    const float angular_speed = 0.1;  // Rate of increase of the angle (rad/s)
    const float radius_step = 0.001;  // Rate of expansion of the radius per step
    const float yaw_rate = 0.005;  // Slow yaw rotation
    
    // Update the angle and radius
    angle += angular_speed;
    r += radius_step;
    
    // Calculate the desired velocity for the spiral motion in x and y
    desiredState->vx = r * cos(angle);
    desiredState->vy = r * sin(angle);
    
    // Keep altitude constant
    desiredState->altitude = FLYING_ALTITUDE;
    
    // Slow yaw rotation to cover the area
    desiredState->yaw_rate = yaw_rate;
}

void spiral_search_3d(cf_state_t *measuredState, cf_state_t *desiredState) {
    static float r = 0.05;  // Start radius of the spiral
    static float angle = 0.0;  // Angular position in radians
    static float altitude_offset = 0.0;  // Offset for altitude oscillation
    static bool ascending = true;  // Direction of altitude change
    const float angular_speed = 0.1;  // Rate of increase of the angle (rad/s)
    const float radius_step = 0.001;  // Rate of expansion of the radius per step
    const float yaw_rate = 0.005;  // Slow yaw rotation
    const float altitude_step = 0.005;  // Change in altitude per step
    const float MAX_ALTITUDE = FLYING_ALTITUDE + 1;
    const float MIN_ALTITUDE = FLYING_ALTITUDE - 1;
    
    // Update the angle and radius
    angle += angular_speed;
    r += radius_step;

    // Calculate the desired velocity for the spiral motion in x and y
    desiredState->vx = r * cos(angle);
    desiredState->vy = r * sin(angle);
    
    // Handle altitude oscillation
    if (ascending) {
        altitude_offset += altitude_step;
        if (desiredState->altitude + altitude_offset > MAX_ALTITUDE) {
            ascending = false;
        }
    } else {
        altitude_offset -= altitude_step;
        if (desiredState->altitude + altitude_offset < MIN_ALTITUDE) {
            ascending = true;
        }
    }

    desiredState->altitude = FLYING_ALTITUDE + altitude_offset;
    
    // Slow yaw rotation to cover the area
    desiredState->yaw_rate = yaw_rate;
}