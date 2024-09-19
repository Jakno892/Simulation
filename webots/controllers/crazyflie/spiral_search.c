#include "spiral_search.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

void spiralSearch(cf_state_t *measuredState, cf_state_t *desiredState, coord_t* spiralCenter)
{
  static int altitudeStepCounter = 0;
  static int altitudeDirection = 1;  // 1 for increasing altitude, -1 for decreasing
  float angle = measuredState->yaw;

  float dx = measuredState->x - spiralCenter->x;
  float dy = measuredState->y - spiralCenter->y;
  float r = sqrt(dx*dx + dy*dy);
  float altitudeCenter = spiralCenter->z;
  
  // Update the radial distance (r) with the fixed radial speed
  r += RADIAL_VELOCITY;
  if (r > MAX_RADIUS)
  {
      // If the spiral reaches max radius, reset to oscillating altitude
      r = MAX_RADIUS;
      
      // Oscillate altitude
      measuredState->altitude += altitudeDirection * RADIAL_VELOCITY;
      if (measuredState->altitude >= altitudeCenter + MAX_ALTITUDE_CHANGE || measuredState->altitude <= altitudeCenter - MAX_ALTITUDE_CHANGE) {
          altitudeDirection *= -1.0f;  // Reverse direction
      }
  }

  // Calculate new angle angle based on radial speed and angular velocity
  angle += ANGULAR_VELOCITY / r;  // Ensure angular velocity decreases as r increases

  // Compute relative x and y based on polar coordinates
  float x_relative = r * cosf(angle);
  float y_relative = r * sinf(angle);
  
  // Set the desired position incrementally in relative coordinates
  desiredState->x = spiralCenter->x + x_relative;
  desiredState->y = spiralCenter->y + y_relative;
  desiredState->z = spiralCenter->z + altitudeDirection*0.01;

  // Set the yaw to always point in the direction of the spiral's movement (tangential)
  desiredState->yaw = angle;
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

float constrain(float value, const float minVal, const float maxVal)
{
  return fminf(maxVal, fmaxf(minVal, value));
}

float calc_distance(coord_t point_1, coord_t point_2)
{
  return sqrt(pow(point_1.x - point_2.x,2) + pow(point_1.y - point_2.y,2) + pow(point_1.z - point_2.z,2));
}

void velocityTransform(cf_state_t* state)
{
  float sinYaw = sin(state->yaw);
  float cosYaw = cos(state->yaw);

  float vx = state->vx;
  float vy = state->vy;

  // Rotate the coordinate system from global coordinates to body fixed coordinates

  state->vx = cosYaw*vx + sinYaw*vy;
  state->vy = -sinYaw*vx + cosYaw*vy;
}

// Transform from world frame to body frame
void transform(cf_state_t* state)
{
  float sinRoll = sin(state->roll);
  float sinPitch = sin(state->pitch); // Sine values of roll, pitch and yaw
  float sinYaw = sin(state->yaw);

  float cosRoll = cos(state->roll);
  float cosPitch = cos(state->pitch); // Cosine values of roll, pitch and yaw
  float cosYaw = cos(state->yaw);

  float x = state->x;
  float y = state->y;
  float z = state->z;

  // Rotate the coordinate system from global coordinates to body fixed coordinates

  state->x = cosYaw*cosPitch*x + sinYaw*cosPitch*y + -sinPitch*z;      
  state->y = (cosYaw*sinPitch*sinRoll - sinYaw*cosRoll)*x + (sinYaw*sinPitch*sinRoll + cosYaw*cosRoll)*y + cosPitch*sinRoll*z;
  state->z = (cosYaw*sinPitch*cosRoll + sinYaw*sinRoll)*x + (sinYaw*sinPitch*cosRoll - cosYaw*sinRoll)*y + cosPitch*cosRoll*z;

  printf("Returning from transform\n");

}

// Transform from body frame to world frame
void inverse_transform(cf_state_t* state)
{
  float sinRoll = sin(state->roll);
  float sinPitch = sin(state->pitch); // Sine values of roll, pitch and yaw
  float sinYaw = sin(state->yaw);

  float cosRoll = cos(state->roll);
  float cosPitch = cos(state->pitch); // Cosine values of roll, pitch and yaw
  float cosYaw = cos(state->yaw);

  float x = state->x;
  float y = state->y;
  float z = state->z;

  // Rotate the coordinate system from bidy fixed coordinates to global coordinates

  state->x = cosYaw*cosPitch*x +
            (cosYaw*sinPitch*sinRoll - sinYaw*cosRoll)*y + 
            (cosYaw*sinPitch*cosRoll + sinYaw*sinRoll)*z;
  
  state->y = sinYaw*cosPitch*x + 
            (sinYaw*sinPitch*sinRoll + cosYaw*cosRoll)*y + 
            (sinYaw*sinPitch*cosRoll - cosYaw*sinRoll)*z;

  state->z = -sinPitch*x + cosPitch*sinRoll*y + cosPitch*cosRoll*z;
}
