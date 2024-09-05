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

void pid_attitude_fixed_height_controller(cf_state_t actual_state, cf_state_t *desired_state, gainsPid_t gains_pid,
                                          float dt, motorPower_t *motorCommands) {
  controlCommands_t control_commands = {0};
  pid_fixed_height_controller(actual_state, desired_state, gains_pid, dt, &control_commands);
  pid_attitude_controller(actual_state, desired_state, gains_pid, dt, &control_commands);
  motor_mixing(control_commands, motorCommands);
}

void pid_velocity_fixed_height_controller(cf_state_t actual_state, cf_state_t *desired_state, gainsPid_t gains_pid,
                                          float dt, motorPower_t *motorCommands) {
  controlCommands_t control_commands = {0};
  pid_horizontal_velocity_controller(actual_state, desired_state, gains_pid, dt);
  pid_fixed_height_controller(actual_state, desired_state, gains_pid, dt, &control_commands);
  pid_attitude_controller(actual_state, desired_state, gains_pid, dt, &control_commands);
  motor_mixing(control_commands, motorCommands);
}

void pid_fixed_height_controller(cf_state_t actual_state, cf_state_t *desired_state, gainsPid_t gains_pid, float dt,
                                 controlCommands_t *control_commands) {
  float altitudeError = desired_state->altitude - actual_state.altitude;
  float altitudeDerivativeError = (altitudeError - pastAltitudeError) / dt;
  control_commands->altitude =
    gains_pid.kp_z * constrain(altitudeError, -1, 1) + gains_pid.kd_z * altitudeDerivativeError + gains_pid.ki_z;

  altitudeIntegrator += altitudeError * dt;
  control_commands->altitude = gains_pid.kp_z * constrain(altitudeError, -1, 1) + gains_pid.kd_z * altitudeDerivativeError +
                               gains_pid.ki_z * altitudeIntegrator + 48;
  pastAltitudeError = altitudeError;
}

void motor_mixing(controlCommands_t control_commands, motorPower_t *motorCommands) {
  // Motor mixing
  motorCommands->m1 = control_commands.altitude - control_commands.roll + control_commands.pitch + control_commands.yaw;
  motorCommands->m2 = control_commands.altitude - control_commands.roll - control_commands.pitch - control_commands.yaw;
  motorCommands->m3 = control_commands.altitude + control_commands.roll - control_commands.pitch + control_commands.yaw;
  motorCommands->m4 = control_commands.altitude + control_commands.roll + control_commands.pitch - control_commands.yaw;
}

void pid_attitude_controller(cf_state_t actual_state, cf_state_t *desired_state, gainsPid_t gains_pid, float dt,
                             controlCommands_t *control_commands) {
  // Calculate errors
  float pitchError = desired_state->pitch - actual_state.pitch;
  float pitchDerivativeError = (pitchError - pastPitchError) / dt;
  float rollError = desired_state->roll - actual_state.roll;
  float rollDerivativeError = (rollError - pastRollError) / dt;
  float yawRateError = desired_state->yaw - actual_state.yaw;

  // PID control
  control_commands->roll = gains_pid.kp_att_rp * constrain(rollError, -1, 1) + gains_pid.kd_att_rp * rollDerivativeError;
  control_commands->pitch = -gains_pid.kp_att_rp * constrain(pitchError, -1, 1) - gains_pid.kd_att_rp * pitchDerivativeError;
  control_commands->yaw = gains_pid.kp_att_y * constrain(yawRateError, -1, 1);

  // Save error for the next round
  pastPitchError = pitchError;
  pastRollError = rollError;
  pastYawRateError = yawRateError;
}

void pid_horizontal_velocity_controller(cf_state_t actual_state, cf_state_t *desired_state, gainsPid_t gains_pid,
                                        float dt) {
  float vxError = desired_state->vx - actual_state.vx;
  float vxDerivative = (vxError - pastVxError) / dt;
  float vyError = desired_state->vy - actual_state.vy;
  float vyDerivative = (vyError - pastVyError) / dt;

  // PID control
  float pitchCommand = gains_pid.kp_vel_xy * constrain(vxError, -1, 1) + gains_pid.kd_vel_xy * vxDerivative;
  float rollCommand = -gains_pid.kp_vel_xy * constrain(vyError, -1, 1) - gains_pid.kd_vel_xy * vyDerivative;

  desired_state->pitch = pitchCommand;
  desired_state->roll = rollCommand;

  // Save error for the next round
  pastVxError = vxError;
  pastVyError = vyError;
}
