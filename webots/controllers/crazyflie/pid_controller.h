#pragma once

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

/*
 *  ...........       ____  _ __
 *  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 *  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 *  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 *
 * @file pid_controller.h
 * Description:  PID controller header file
 * Author:       Kimberly McGuire (Bitcraze AB)
 */

#include "spiral_search.h"

float constrain(float value, const float minVal, const float maxVal);
void init_pid_attitude_fixed_height_controller();

void pid_attitude_fixed_height_controller(cf_state_t* measuredState, cf_state_t *desiredState, pidParams_t pidParams,
                                          float dt, motorPower_t *motorCommands);

void pid_velocity_fixed_height_controller(cf_state_t* measuredState, cf_state_t *desiredState, pidParams_t pidParams,
                                          float dt, motorPower_t *motorCommands);

void pid_fixed_height_controller(cf_state_t* measuredState, cf_state_t *desiredState, pidParams_t pidParams, float dt,
                                 control_t *controlCommands);

void motor_mixing(control_t controlCommands, motorPower_t *motorCommands);

void pid_attitude_controller(cf_state_t* measuredState, cf_state_t *desiredState, pidParams_t pidParams, float dt,
                             control_t *controlCommands);

void pid_horizontal_velocity_controller(cf_state_t* measuredState, cf_state_t *desiredState, pidParams_t pidParams,
                                        float dt);