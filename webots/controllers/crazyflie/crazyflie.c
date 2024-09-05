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
#define MAX_RADIUS 1.0

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


  bool autonomous = false;
  bool recording = false;
  bool network = false;
  FILE *groundTruthData;
  float dt_counter = 0;
  float frameRate = 1/10;
  int image_counter = 1;

  Node* path[1000]; // Array to store the path
  int path_counter;
  int pathLength;
  float max_value;

  // Wait for 2 seconds
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }
  // Initialize variables
  cf_state_t measuredState = {0};
  cf_state_t desiredState = {0};
  float past_v_x = 0;
  float past_v_y = 0;
  float past_v_z = 0;
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

  float height_desired = FLYING_ALTITUDE;

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

  coord_t prevState = {0};
  cf_state_t currentState = {0};

  currentState.r_max = 5;
  const float r_step = 0.05; // Step size per 
  const float yaw = 0.005; // Rate of angular increase relative to radial (m/rad)

  while (wb_robot_step(timestep) != -1)
  {
    const float dt = wb_robot_get_time() - pastTime;

    dt_counter += dt;
    
    // Ground truth position measurements
    coord_t groundTruth;
    groundTruth.x = wb_gps_get_values(gps)[0];
    groundTruth.y = wb_gps_get_values(gps)[1];
    groundTruth.z = wb_gps_get_values(gps)[2];

    // Get measurements
    measuredState.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    measuredState.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    measuredState.yaw = wb_gyro_get_values(gyro)[2];
    measuredState.altitude = wb_altimeter_get_value(altimeter);
    float actualYaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    float actual_v_x = (groundTruth.x - prevState.x) / dt;
    float actual_v_y = (groundTruth.y - prevState.y) / dt;
    
    // Get body fixed velocities
    // measuredState.vx =  actual_v_x * cos(actualYaw) + actual_v_y * sin(actualYaw);
    // measuredState.vy = -actual_v_x * sin(actualYaw) + actual_v_y * cos(actualYaw);

    float a_x = wb_accelerometer_get_values(accelerometer)[0]; // Acceleration
    float a_y = wb_accelerometer_get_values(accelerometer)[1]; // Acceleration
    float a_z = wb_accelerometer_get_values(accelerometer)[2]; // Acceleration

    // Calculate velocity and position
    float v_x = past_v_x + a_x*dt;
    float v_y = past_v_y + a_y*dt;
    float v_z = past_v_z + a_z*dt;

    measuredState.vx =  v_x * cos(actualYaw) + v_y * sin(actualYaw);
    measuredState.vy = -v_x * sin(actualYaw) + v_y * cos(actualYaw);
    printf("Measured v: (%f,%f)\n", v_x, v_x);
    printf("Actual v:   (%f,%f)\n", actual_v_x, actual_v_y);
    printf("--------------------------\n");

    x_global += v_x*dt;
    y_global += v_y*dt;
    z_global += v_z*dt;

    measuredState.x = x_global;
    measuredState.y = y_global;
    measuredState.z = z_global;

    // printf("Measured position: (%f,%f)\n", x_global,y_global);
    // printf("Actual position:   (%f,%f)\n", groundTruth.x,groundTruth.y);
    // printf("--------------------------\n");
    coord_t opticalPoint = {2.15,0.3,1.5};



    // Initialize values
    desiredState.roll = 0;
    desiredState.pitch = 0;
    desiredState.vx = 0;
    desiredState.vy = 0;
    desiredState.yaw = 0;
    desiredState.altitude = FLYING_ALTITUDE;

    currentState.dt = dt;
    currentState.x = x_global;
    currentState.y = y_global;
    currentState.z = z_global;
    currentState.vx = actual_v_x;
    currentState.vy = actual_v_y;
    currentState.yaw = actualYaw;

    if(autonomous)
    {
      int key = wb_keyboard_get_key();
      while (key > 0)
      {
        switch (key)
        {
          case 'D':
            printf("---Returning to manual control---\n");
            autonomous = false;
            break;
          case 'N':
            if(!network)
            {
              printf("---Light source detected!---\n");
              network = true;
              path_counter = 0;
              pathLength = 0;
              
              // Starting position (x, y, z, yaw)
              coord_t start = {currentState.x, currentState.y, currentState.z, currentState.yaw*dt*180/M_PI};
              coord_t goal = {opticalPoint.x - 0.3, opticalPoint.y, opticalPoint.z, 0}; // Goal position
              
              float totalCost = AStar(start, goal, path, &pathLength);
              printPath(path, pathLength);
            }
            break;
          case 'O':
            if(network)
            {
              printf("---Light source lost. Returning to search---\n");
              network = false;
            }
            break;
          case 'R':
            if(!recording)
            {
              printf("---Recording camera stream---\n");
              recording = true;
              groundTruthData = fopen("../../data_collection/data.csv","w");
            }
            break;
          case 'X':
            if(recording)
            {
              printf("---Aborted recording---\n");
              recording = false;
              fclose(groundTruthData);
            }
            break;
        }
        key = wb_keyboard_get_key();
      }

      if (network)
      {
        if (path_counter != pathLength)
        {
          desiredState.vx = path[path_counter]->coord.x; // - currentState.x;
          desiredState.vy = path[path_counter]->coord.y; // - currentState.y;
          desiredState.altitude *= path[path_counter]->coord.z*dt/FLYING_ALTITUDE; // - currentState.z)*dt;
          desiredState.yaw = path[path_counter]->coord.yaw*M_PI/180 - currentState.yaw;
          
          if(desiredState.x == path[path_counter]->coord.x && 
             desiredState.y == path[path_counter]->coord.x &&
             desiredState.z == path[path_counter]->coord.x &&
             desiredState.yaw == path[path_counter]->coord.yaw)
          {
            path_counter++;
          }
        }
        else
        {
          desiredState.vx = 0;
          desiredState.vy = 0;
          desiredState.altitude = 0;
          desiredState.yaw = 0;
        }
      }
      else
      {
        desiredState.vx = 0;
        desiredState.vy = 0;
        desiredState.altitude = FLYING_ALTITUDE;
        desiredState.yaw = 0;
        // spiral_search(&desiredState, r_step, yaw);
      }      
    }
    else
    {
      int key = wb_keyboard_get_key();
      while (key > 0)
      {
        switch (key)
        {
          case WB_KEYBOARD_UP:
            desiredState.vx = +0.5;
            break;
          case WB_KEYBOARD_DOWN:
            desiredState.vx = -0.5;
            break;
          case WB_KEYBOARD_RIGHT:
            desiredState.vy = -0.5;
            break;
          case WB_KEYBOARD_LEFT:
            desiredState.vy = +0.5;
            break;
          case 'Q':
            desiredState.yaw = 1.0;
            break;
          case 'E':
            desiredState.yaw = -1.0;
            break;
          case 'W':
            desiredState.altitude += 10*dt;
            break;
          case 'S':
            desiredState.altitude -= 10*dt;
            break;
          case 'A':
            if(!autonomous)
            {
              printf("---Initiating spiral search protocol---\n");
              autonomous = true;
            }
            break;
          case 'R':
            if(!recording)
            {
              printf("---Recording camera stream---\n");
              recording = true;
              groundTruthData = fopen("../../data_collection/data.csv","w");
            }
            break;
          case 'X':
            if(recording)
            {
              printf("---Aborted recording---\n");
              recording = false;
              fclose(groundTruthData);
            }
            break;
        }
        key = wb_keyboard_get_key();
      }
      
    }
    
    // Example how to get sensor data
    // range_front_value = wb_distance_sensor_get_value(range_front));
    // const unsigned char *image = wb_camera_get_image(camera);

    float pointDistance = sqrt(pow(opticalPoint.x - groundTruth.x,2) + pow(opticalPoint.y - groundTruth.y,2));

    if(recording && dt_counter >= frameRate)
    {
      // printf("1\n");
      char file_template[48];
      // printf("2\n");
      // sn
      sprintf(file_template,"../../data_collection/train/recording_%d.png",image_counter);
      // printf("3\n");
      // printf("filename: %s", *filename);
      const char* filename = file_template;
      wb_camera_save_image(camera, filename,0);
      fprintf(groundTruthData, "%.2f\n", roundf(100*pointDistance)/100);
      printf("Distance to point:%.2f",calc_distance(opticalPoint, groundTruth));
      dt_counter = 0;
      image_counter += 1;
    }

    // PID velocity controller with fixed height
    pid_velocity_fixed_height_controller(measuredState, &desiredState, gainsPid, dt, &motorPower);
    // pid_controller(measuredState, &desiredState, gainsPid, dt, &motorPower);
    // Setting motorspeed
    wb_motor_set_velocity(m1_motor, -motorPower.m1);
    wb_motor_set_velocity(m2_motor, motorPower.m2);
    wb_motor_set_velocity(m3_motor, -motorPower.m3);
    wb_motor_set_velocity(m4_motor, motorPower.m4);

    // Save past time for next time step
    pastTime = wb_robot_get_time();
    past_v_x = v_x;
    past_v_y = v_y;
    past_v_z = v_z;
    prevState = groundTruth;
  };

  // Free allocated memory
  for (int i = 0; i < pathLength; i++)
  {
    free(path[i]);
  }

  wb_robot_cleanup();

  return 0;
}