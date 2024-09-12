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
 * @file crazyflie_controller.c
 * Description: Controls the crazyflie in webots
 * Author:      Kimberly McGuire (Bitcraze AB)
 */

#include <math.h>
#include <stdio.h>
#include <time.h>

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/accelerometer.h>
#include <webots/altimeter.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>


// Add external controller
#include "pid_controller.h"
#include "spiral_search.h"
#include "path_planner.h"

#define FLYING_ALTITUDE 1.5

int main(int argc, char **argv) {
  wb_robot_init();

  const int timestep = (int)wb_robot_get_basic_time_step();

  // Initialize motors
  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_velocity(m1_motor, -1.0);
  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_velocity(m2_motor, 1.0);
  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_velocity(m3_motor, -1.0);
  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
  wb_motor_set_position(m4_motor, INFINITY);
  wb_motor_set_velocity(m4_motor, 1.0);

  // Initialize sensors
  WbDeviceTag imu = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, timestep);
  WbDeviceTag altimeter = wb_robot_get_device("altimeter");
  wb_altimeter_enable(altimeter, timestep);

  // Wait for 2 seconds
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }
  // Initialize variables
  cf_state_t measuredState = {0};
  cf_state_t previousState = {0};
  cf_state_t desiredState = {0};
  float pastTime = wb_robot_get_time();
  
  float x_global = 0;
  float y_global = 0;
  float z_global = 0;

  // Initialize PID gains.
  gainsPid_t gainsPid;
  gainsPid.kp_att_y = 1;
  gainsPid.kd_att_y = 0.5;
  gainsPid.kp_att_rp = 0.5;
  gainsPid.kd_att_rp = 0.1;
  gainsPid.kp_vel_xy = 2;
  gainsPid.kd_vel_xy = 0.5;
  gainsPid.kp_z = 10;
  gainsPid.ki_z = 5;
  gainsPid.kd_z = 5;
  init_pid_attitude_fixed_height_controller();

  // Initialize struct for motor power
  motorPower_t motorPower;

  printf("\n");

  printf("====== Controls =======\n");

  printf(" The Crazyflie can be controlled from your keyboard!\n");
  printf(" All controllable movement is in body coordinates\n");
  printf("- Use the up, back, right and left button to move in the horizontal plane\n");
  printf("- Use Q and E to rotate around yaw\n ");
  printf("- Use W and S to go up and down\n");
  printf("- Use A and D to enable/disable the autonomous spiral search protocol\n");

  float height_desired = FLYING_ALTITUDE;

  while (wb_robot_step(timestep) != -1)
  {
    const float dt = wb_robot_get_time() - pastTime;
    
    // Ground truth position measurements
    cf_state_t groundTruth;
    groundTruth.x = wb_gps_get_values(gps)[0];
    groundTruth.y = wb_gps_get_values(gps)[1];
    groundTruth.z = wb_gps_get_values(gps)[2];
    groundTruth.vx = (groundTruth.x - previousState.x) / dt;
    groundTruth.vy = (groundTruth.y - previousState.y) / dt;
    groundTruth.vz = (groundTruth.z - previousState.z) / dt;
    groundTruth.ax = (groundTruth.vx - previousState.vx) / dt;
    groundTruth.ay = (groundTruth.vy - previousState.vy) / dt;
    groundTruth.az = (groundTruth.vz - previousState.vz) / dt;


    // Get measurements
    measuredState.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    measuredState.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    measuredState.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    measuredState.yaw_rate = wb_gyro_get_values(gyro)[2];
    // measuredState.altitude = wb_gps_get_values(gps)[2];
    measuredState.altitude = wb_altimeter_get_value(altimeter);

    // Get the body fixed coordinates
    measuredState.ax = wb_accelerometer_get_values(accelerometer)[0]; // Acceleration
    measuredState.ay = wb_accelerometer_get_values(accelerometer)[1]; // Acceleration
    measuredState.az = wb_accelerometer_get_values(accelerometer)[2]; // Acceleration
    
    // Initialize values
    desiredState.roll = 0;
    desiredState.pitch = 0;
    desiredState.vx = 0;
    desiredState.vy = 0;
    // desiredState.yaw = 0;
    desiredState.yaw_rate = 0;
    desiredState.altitude = FLYING_ALTITUDE;

    float forward_desired = 0;
    float sideways_desired = 0;
    float yaw_desired = 0;
    float height_diff_desired = 0;

    int key = wb_keyboard_get_key();
    while (key > 0)
    {
    switch (key)
    {
        case WB_KEYBOARD_UP:
            forward_desired = +0.5;
            break;
        case WB_KEYBOARD_DOWN:
            forward_desired = -0.5;
            break;
        case WB_KEYBOARD_RIGHT:
            sideways_desired = -0.5;
            break;
        case WB_KEYBOARD_LEFT:
            sideways_desired = +0.5;
            break;
        case 'Q':
            yaw_desired = 1.0;
            break;
        case 'E':
            yaw_desired = -1.0;
            break;
        case 'W':
            height_diff_desired = 0.1;
            break;
        case 'S':
            height_diff_desired = 0.1;
            break;
    }
    key = wb_keyboard_get_key();
    }

      
    height_desired += height_diff_desired * dt;
    // PID velocity controller with fixed height
    desiredState.vy = sideways_desired;
    desiredState.vx = forward_desired;
    desiredState.altitude = height_desired;
    desiredState.yaw_rate = yaw_desired;
          
    // PID velocity controller with fixed height
    pid_velocity_fixed_height_controller(measuredState, &desiredState, gainsPid, dt, &motorPower);

    // Setting motorspeed
    wb_motor_set_velocity(m1_motor, -motorPower.m1);
    wb_motor_set_velocity(m2_motor, motorPower.m2);
    wb_motor_set_velocity(m3_motor, -motorPower.m3);
    wb_motor_set_velocity(m4_motor, motorPower.m4);

    // Save past time for next time step
    pastTime = wb_robot_get_time();

    previousState = measuredState;
  };

  wb_robot_cleanup();

  return 0;
}