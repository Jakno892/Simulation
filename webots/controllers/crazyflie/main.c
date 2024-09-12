/*
  Modified from the crazyflie model.
  Link: 
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
  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor"); // Front left
  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_velocity(m1_motor, -1.0);
  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor"); // Front right
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_velocity(m2_motor, 1.0);
  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor"); // Back right
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_velocity(m3_motor, -1.0);
  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor"); // Back left
  wb_motor_set_position(m4_motor, INFINITY);
  wb_motor_set_velocity(m4_motor, 1.0);

  // Initialize sensors
  WbDeviceTag imu = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag tofsensor = wb_robot_get_device("tofSensor");
  wb_distance_sensor_enable(tofsensor, timestep);
  
  wb_keyboard_enable(timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  // WbDeviceTag opticalflow = wb_robot_get_device("opticalFlowSensor");
  // wb_camera_enable(opticalflow, timestep);

  enum States
  {
    START,
    SEARCH,
    ENGAGE,
    LAND
  };
  int state;

  coord_t opticalPoint = {2.15,0.3,1.5}; // Location of the optical point in global coordinates

  // Initiate camera variables
  bool autonomous = false;
  bool recording = false;
  bool network = false;
  FILE *groundTruthData;
  float dt_counter = 0;
  float frameRate = 1/10;
  int image_counter = 1;

  // Initiate control variables
  Node* path[1000]; // Array to store the path
  int path_counter;
  int pathLength;
  float max_value;
  int node_counter;

  // Wait for 2 seconds
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }
  // Initialize variables
  cf_state_t measuredState = {0};
  cf_state_t desiredState = {0};

  coord_t previousState = {0};

  float pastTime = wb_robot_get_time();
  
  float x_global = 0;
  float y_global = 0;
  float z_global = 0;

  // Initialize PID gains.
  pidParams_t pidParams;
  pidParams.kp_att_y = 1;
  pidParams.kd_att_y = 0.5;
  pidParams.kp_att_rp = 0.5;
  pidParams.kd_att_rp = 0.1;
  pidParams.kp_vel_xy = 2;
  pidParams.kd_vel_xy = 0.5;
  pidParams.kp_z = 10;
  pidParams.ki_z = 5;
  pidParams.kd_z = 5;
  init_pid_attitude_fixed_height_controller();

  // Initialize struct for motor power
  motorPower_t motorPower;

  float height_desired = FLYING_ALTITUDE;

  const float r_step = 0.05; // Step size per 
  const float yaw = 0.005; // Rate of angular increase relative to radial (m/rad)

  // Starting position (x, y, z, yaw)
  coord_t start = {0, 0, 0, 0};
  coord_t goal = {opticalPoint.x - 0.3, opticalPoint.y, opticalPoint.z, 180}; // Goal position
  
  float totalCost = AStar(start, goal, path, &pathLength);
  printPath(path, pathLength);

  while (wb_robot_step(timestep) != -1)
  {
    const float dt = wb_robot_get_time() - pastTime;

    dt_counter += dt;
    
    // Ground truth position measurements
    coord_t groundTruth;
    groundTruth.x = wb_gps_get_values(gps)[0];
    groundTruth.y = wb_gps_get_values(gps)[1];

    /*
        |------------------------------------------------------------|
        |------------------- GET MEASUREMENTS -----------------------|
        |------------------------------------------------------------|
    */ 
    measuredState.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    measuredState.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    measuredState.yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    measuredState.yaw_rate = wb_gyro_get_values(gyro)[2];
    measuredState.altitude = wb_distance_sensor_get_value(tofsensor);
   
    // Calculate velocity and position  
    measuredState.vx = (groundTruth.x - previousState.x) / dt;
    measuredState.vy = (groundTruth.y - previousState.y) / dt;

    measuredState.x += measuredState.vx*dt;
    measuredState.y += measuredState.vy*dt;

    // Initialize values
    desiredState.roll = 0;
    desiredState.pitch = 0;
    desiredState.vx = 0;
    desiredState.vy = 0;
    // desiredState.yaw = 0;
    desiredState.yaw_rate = 0;
    desiredState.altitude = FLYING_ALTITUDE;


    //  |------------------------------------------------------------|
    //  |-------------------- STATE  MACHINE ------------------------|
    //  |------------------------------------------------------------|

    /*
    if(autonomous)
    {

      // Check for keyboard input
      int key = wb_keyboard_get_key();
      while (key > 0)
      {
        switch (key)
        {
          // Disable autonomous mode
          case 'D': 
            printf("---Returning to manual control---\n");
            autonomous = false;
            break;

          // Simulate network activation, i.e. optical point found
          case 'N':
            if(!network)
            {
              printf("---Light source detected!---\n");

              state = ENGAGE;
              path_counter = 0;
              pathLength = 0;
              
              // Starting position (x, y, z, yaw)
              coord_t start = {measuredState.x, measuredState.y, measuredState.z, measuredState.yaw*dt*180/M_PI};
              coord_t goal = {opticalPoint.x - 0.3, opticalPoint.y, opticalPoint.z, 0}; // Goal position
              
              float totalCost = AStar(start, goal, path, &pathLength);
              printPath(path, pathLength);
            }
            break;
          
          // Simulate network deactivation, i.e. optical point lost
          case 'O':
            if(network)
            {
              printf("---Light source lost. Returning to search---\n");
              network = false;
            }
            break;

          // Record camera images
          case 'R':
            if(!recording)
            {
              printf("---Recording camera stream---\n");
              recording = true;
              groundTruthData = fopen("../../data_collection/data.csv","w");
            }
            break;

          // Cancel camera recording
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

      // State machine
      switch (state)
      {
      case START:
        if (measuredState.altitude == FLYING_ALTITUDE)
        {
          state = SEARCH;
        }
        break;

      case SEARCH:
        // spiral_search(&measuredState, &desiredState, r_step, yaw);

        break;

      case ENGAGE:
        if (path_counter != pathLength)
        {
          desiredState.vx = path[path_counter]->coord.x; // - measuredState.x;
          desiredState.vy = path[path_counter]->coord.y; // - measuredState.y;
          desiredState.altitude *= path[path_counter]->coord.z*dt/FLYING_ALTITUDE; // - measuredState.z)*dt;
          desiredState.yaw = path[path_counter]->coord.yaw*M_PI/180 - measuredState.yaw;
          
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
        break;

      case LAND:
        // Starting position (x, y, z, yaw)
        coord_t start = {measuredState.x, measuredState.y, measuredState.z, measuredState.yaw*dt*180/M_PI};
        coord_t goal = {0, 0, 0.1, 0}; // Goal position
        
        float totalCost = AStar(start, goal, path, &pathLength);
        printPath(path, pathLength);
        break;
      }   
    }
    else
    {
    */
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
            /*
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
          */
        }
        key = wb_keyboard_get_key();
      }

      if (calc_distance(groundTruth, opticalPoint) < 0.1)
      {
        printf("Reached end of path \n");
        sideways_desired = 0;
        forward_desired = 0;
        height_diff_desired = 0;
      }
      else if (calc_distance(groundTruth, path[node_counter]->coord) < 0.1)
      {
          printf("Reached node %d\n", node_counter + 1);
          if (node_counter < pathLength - 1)
          {
            node_counter++;
          }
          // Update desired state to the next waypoint
          sideways_desired = 0.5*(path[node_counter]->coord.y - groundTruth.y);
          forward_desired = 0.5*(path[node_counter]->coord.x - groundTruth.x);
          height_diff_desired = 0.1*(path[node_counter]->coord.z - groundTruth.z);
      }
      
      height_desired += height_diff_desired * dt;

      desiredState.vy = sideways_desired;
      desiredState.vx = forward_desired;
      desiredState.altitude = height_desired;
      desiredState.yaw_rate = yaw_desired;
      
    // }
    
    // Example how to get sensor data
    // const unsigned char *image = wb_camera_get_image(camera);

    /*
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
      printf("Distance to point:%.2f",pointDistance);
      dt_counter = 0;
      image_counter += 1;
    }
    */

    // PID velocity controller with fixed height
    pid_velocity_fixed_height_controller(measuredState, &desiredState, pidParams, dt, &motorPower);
    // Run the PID controller
    // pid_controller(&measuredState, &desiredState, pidParams, dt, &motorPower);
    
    // Setting motorspeed
    wb_motor_set_velocity(m1_motor, -motorPower.m1);
    wb_motor_set_velocity(m2_motor, motorPower.m2);
    wb_motor_set_velocity(m3_motor, -motorPower.m3);
    wb_motor_set_velocity(m4_motor, motorPower.m4);

    // Save past time for next time step
    pastTime = wb_robot_get_time();
    previousState = groundTruth;
    // previousState = measuredState;
  };

  // Free allocated memory
  for (int i = 0; i < pathLength; i++)
  {
    free(path[i]);
  }

  wb_robot_cleanup();

  return 0;
}