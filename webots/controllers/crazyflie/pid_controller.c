/*
 * Copyright 2022 Bitcraze AB
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 *  ...........       ____  _ __
 *  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 *  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 *  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 *
 * @file pid_controller.c
 * Description: A simple PID controller for attitude,
 *          height and velocity  control of an quadcopter
 * Author:      Kimberly McGuire (Bitcraze AB)

 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pid_controller.h"

float constrain(float value, const float minVal, const float maxVal) {
  return fminf(maxVal, fmaxf(minVal, value));
}

float pastAltitudeError, pastPitchError, pastRollError, pastYawRateError;
float pastVxError, pastVyError;
float altitudeIntegrator;

void init_pid_attitude_fixed_height_controller() {
  pastAltitudeError = 0;
  pastYawRateError = 0;
  pastPitchError = 0;
  pastRollError = 0;
  pastVxError = 0;
  pastVyError = 0;
  altitudeIntegrator = 0;
}

void pid_attitude_fixed_height_controller(cf_state_t* measuredState, cf_state_t *desiredState, pidParams_t pidParams,
                                          float dt, motorPower_t *motorCommands) {
  control_t controlCommands = {0};
  pid_fixed_height_controller(measuredState, desiredState, pidParams, dt, &controlCommands);
  pid_attitude_controller(measuredState, desiredState, pidParams, dt, &controlCommands);
  motor_mixing(controlCommands, motorCommands);
}

void pid_velocity_fixed_height_controller(cf_state_t* measuredState, cf_state_t *desiredState, pidParams_t pidParams,
                                          float dt, motorPower_t *motorCommands) {
  control_t controlCommands = {0};
  pid_horizontal_velocity_controller(measuredState, desiredState, pidParams, dt);
  pid_fixed_height_controller(measuredState, desiredState, pidParams, dt, &controlCommands);
  pid_attitude_controller(measuredState, desiredState, pidParams, dt, &controlCommands);
  motor_mixing(controlCommands, motorCommands);
}

void pid_fixed_height_controller(cf_state_t* measuredState, cf_state_t *desiredState, pidParams_t pidParams, float dt,
                                 control_t *controlCommands) {
  float altitudeError = desiredState->altitude - measuredState->altitude;
  float altitudeDerivativeError = (altitudeError - pastAltitudeError) / dt;
  controlCommands->altitude =
    pidParams.kp_z * constrain(altitudeError, -1, 1) + pidParams.kd_z * altitudeDerivativeError + pidParams.ki_z;

  altitudeIntegrator += altitudeError * dt;
  controlCommands->altitude = pidParams.kp_z * constrain(altitudeError, -1, 1) + pidParams.kd_z * altitudeDerivativeError +
                               pidParams.ki_z * altitudeIntegrator + 48;
  pastAltitudeError = altitudeError;
}

void motor_mixing(control_t controlCommands, motorPower_t *motorCommands) {
  // Motor mixing
  motorCommands->frontLeft = controlCommands.altitude - controlCommands.roll + controlCommands.pitch + controlCommands.yaw;
  motorCommands->rearLeft = controlCommands.altitude - controlCommands.roll - controlCommands.pitch - controlCommands.yaw;
  motorCommands->rearRight = controlCommands.altitude + controlCommands.roll - controlCommands.pitch + controlCommands.yaw;
  motorCommands->frontRight = controlCommands.altitude + controlCommands.roll + controlCommands.pitch - controlCommands.yaw;
}

void pid_attitude_controller(cf_state_t* measuredState, cf_state_t *desiredState, pidParams_t pidParams, float dt,
                             control_t *controlCommands) {
  // Calculate errors
  float pitchError = desiredState->pitch - measuredState->pitch;
  float pitchDerivativeError = (pitchError - pastPitchError) / dt;
  float rollError = desiredState->roll - measuredState->roll;
  float rollDerivativeError = (rollError - pastRollError) / dt;
  float yawRateError = desiredState->yaw_rate - measuredState->yaw_rate;

  // PID control
  controlCommands->roll = pidParams.kp_att_rp * constrain(rollError, -1, 1) + pidParams.kd_att_rp * rollDerivativeError;
  controlCommands->pitch = -pidParams.kp_att_rp * constrain(pitchError, -1, 1) - pidParams.kd_att_rp * pitchDerivativeError;
  controlCommands->yaw = pidParams.kp_att_y * constrain(yawRateError, -1, 1);

  // Save error for the next round
  pastPitchError = pitchError;
  pastRollError = rollError;
  pastYawRateError = yawRateError;
}

void pid_horizontal_velocity_controller(cf_state_t* measuredState, cf_state_t *desiredState, pidParams_t pidParams,
                                        float dt) {
  float vxError = desiredState->vx - measuredState->vx;
  float vxDerivative = (vxError - pastVxError) / dt;
  float vyError = desiredState->vy - measuredState->vy;
  float vyDerivative = (vyError - pastVyError) / dt;

  // PID control
  float pitchCommand = pidParams.kp_vel_xy * constrain(vxError, -1, 1) + pidParams.kd_vel_xy * vxDerivative;
  float rollCommand = -pidParams.kp_vel_xy * constrain(vyError, -1, 1) - pidParams.kd_vel_xy * vyDerivative;

  desiredState->pitch = pitchCommand;
  desiredState->roll = rollCommand;

  // Save error for the next round
  pastVxError = vxError;
  pastVyError = vyError;
}
