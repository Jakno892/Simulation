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
// #include "pid_controller.h"
#include "spiral_search.h"
#include "path_planner.h"

#define MAX_RADIUS 1.0

typedef struct sensors
{
    WbDeviceTag imu;
    WbDeviceTag gyro;
    WbDeviceTag accelerometer;
    WbDeviceTag tofsensor;
    WbDeviceTag camera;
    WbDeviceTag gps;
    // WbDeviceTag opticalflow;
} sensor_t;

typedef struct motors
{
    WbDeviceTag frontLeft;
    WbDeviceTag rearLeft;
    WbDeviceTag rearRight;
    WbDeviceTag frontRight;
} motor_t;

void initMotors(motor_t* motors)
{
  // Front left
  wb_motor_set_position(motors->frontLeft, INFINITY);
  wb_motor_set_velocity(motors->frontLeft, -1.0);
  // Front right
  wb_motor_set_position(motors->rearLeft, INFINITY);
  wb_motor_set_velocity(motors->rearLeft, 1.0);
  // Rear left
  wb_motor_set_position(motors->rearRight, INFINITY);
  wb_motor_set_velocity(motors->rearRight, -1.0);
  // Rear right
  wb_motor_set_position(motors->frontRight, INFINITY);
  wb_motor_set_velocity(motors->frontRight, 1.0);
}

void initSensors(sensor_t* sensors, const int timestep)
{
  // Initialize sensors

  wb_inertial_unit_enable(sensors->imu, timestep);
  wb_accelerometer_enable(sensors->accelerometer, timestep);
  wb_gyro_enable(sensors->gyro, timestep);
  wb_camera_enable(sensors->camera, timestep);
  wb_distance_sensor_enable(sensors->tofsensor, timestep);  
  wb_gps_enable(sensors->gps, timestep);
  // wb_camera_enable(opticalflow, timestep);
}

// Collect sensor measurements
void measurementUpdate(sensor_t* sensors, cf_state_t* measuredState, coord_t* previousState, const float dt)
{
  measuredState->roll = wb_inertial_unit_get_roll_pitch_yaw(sensors->imu)[0];
  measuredState->pitch = wb_inertial_unit_get_roll_pitch_yaw(sensors->imu)[1];
  measuredState->yaw = wb_inertial_unit_get_roll_pitch_yaw(sensors->imu)[2];
  measuredState->yaw_rate = wb_gyro_get_values(sensors->gyro)[2];
  // measuredState->altitude = wb_distance_sensor_get_value(sensors->tofsensor);
  // measuredState->altitude *= cos(measuredState->pitch)*cos(measuredState->roll);
  measuredState->altitude = wb_gps_get_values(sensors->gps)[2];

  // Calculate velocity and position 

  // The Flow deck measures horizontal velocity through optical flow
  // Due to no pre-made opticalflow sensors this is simulated through
  // GPS measurements

  float pos_x = wb_gps_get_values(sensors->gps)[0];
  float pos_y = wb_gps_get_values(sensors->gps)[1];

  measuredState->vx = (pos_x - previousState->x) / dt;
  measuredState->vy = (pos_y - previousState->y) / dt;

  measuredState->x += measuredState->vx*dt; // Position is (re)calculated
  measuredState->y += measuredState->vy*dt; // using velocity.
}

int main(int argc, char **argv) {
  wb_robot_init();

  const int timestep = (int)wb_robot_get_basic_time_step();
  wb_keyboard_enable(timestep);

  // Initialize motors
  motor_t motors = 
  {
    .frontLeft = wb_robot_get_device("m1_motor"), // Front left
    .rearLeft = wb_robot_get_device("m2_motor"), // Rear left
    .rearRight = wb_robot_get_device("m3_motor"), // Rear right, 
    .frontRight = wb_robot_get_device("m4_motor")  // Front right
  };
  initMotors(&motors);

  // Initialize sensors
  sensor_t sensors = 
  {
    .imu = wb_robot_get_device("inertial_unit"),
    .gyro = wb_robot_get_device("gyro"),
    .accelerometer = wb_robot_get_device("accelerometer"),
    .tofsensor = wb_robot_get_device("tofSensor"),
    .camera = wb_robot_get_device("camera"),
    .gps = wb_robot_get_device("gps")
    // .opticalflow = wb_robot_get_device("opticalFlowSensor")
  };
  initSensors(&sensors, timestep);

  enum States
  {
    START,
    SEARCH,
    ENGAGE,
    LAND,
    MISSION_COMPLETE
  };
  int state = 0;

  coord_t opticalPoint = {2.15,0.3,1.5}; // Location of the optical point in global coordinates

  // Initialize camera variables
  bool autonomous = false;
  bool recording = false;
  bool network = false;
  FILE *groundTruthData;
  float dt_counter = 0;
  float frameRate = 1/10;
  int image_counter = 1;

  // Initialize path planner variables
  Node* path[1000]; // Preallocate memory for path nodes
  Obstacle* obstacles[1000]; // Preallocate memory for obstacles
  int pathLength = 0; // Initialize number of nodes in path
  int numObstacles = 0; // Initialize number of obstacles
  int node_counter = 1; // Keeps count of which node in the path the MAV is aiming for during ENGAGE state

  // Wait for 2 seconds
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }
  
  // Initialize state variables
  cf_state_t measuredState = {0};
  cf_state_t desiredState = {.altitude = FLYING_ALTITUDE};
  cf_state_t previousState = {0};

  // Initialize PID gains.
  controlError_t altitudeError = {.kp = 10, .ki = 1.5, .kd = 20, .integrator = 0, .previousError = 0}; // Well tuned
  controlError_t rollError = {.kp = 2, .ki = 0, .kd = 3, .integrator = 0, .previousError = 0};
  controlError_t pitchError = {.kp = 2, .ki = 0, .kd = 3, .integrator = 0, .previousError = 0};
  controlError_t yawError = {.kp = 2, .ki = 0, .kd = 1, .integrator = 0, .previousError = 0}; // Well tuned
  controlError_t vxError = {.kp = 1, .ki = 0, .kd = 2, .integrator = 0, .previousError = 0};
  controlError_t vyError = {.kp = 2, .ki = 0, .kd = 1, .integrator = 0, .previousError = 0};

  float pastTime = wb_robot_get_time();

  // Initialize struct for motor power
  motorPower_t motorPower;

  float height_desired = FLYING_ALTITUDE;
  float prev_altitude = 0;

  // Starting and goal positions (x, y, z, yaw)
  coord_t start = {measuredState.x, measuredState.y, measuredState.altitude, measuredState.yaw};
  coord_t goal = {opticalPoint.x - 0.3, opticalPoint.y, opticalPoint.z, 0}; // Goal position
  Obstacle obstacle = // Define an obstacle
  {
    .min={0.6, -1.2, 0},
    .max={1.0, 0.4, 4},
    .margin=0.3
  };
  obstacles[0] = &obstacle;
  numObstacles = 1;
  float totalCost = AStar(start, goal, path, &pathLength, &obstacles, numObstacles);
  // printPath(path, pathLength);
  // drawPath(path, pathLength, obstacles, numObstacles);

  while (wb_robot_step(timestep) != -1)
  {
    const float dt = wb_robot_get_time() - pastTime;

    dt_counter += dt; // Sums up time passage for camera recording frame rate calculations
    
    // Collect sensor measurements |    
    measurementUpdate(&sensors, &measuredState, &previousState, dt);

    //  |------------------------------------------------------------|
    //  |-------------------- STATE  MACHINE ------------------------|
    //  |------------------------------------------------------------|

        // State machine
    switch (state)
    {
      case START:
        if(fabs(measuredState.altitude - FLYING_ALTITUDE) < 0.01 && fabs(measuredState.altitude - prev_altitude) < 0.005)
        {
          state = ENGAGE;
          // state = LAND;
          printf("########################## \n");
          printf("Entering search pattern \n");
          printf("########################## \n");
        }
        break;

      case SEARCH:  // Add spiral search logic here
        // Choose between 2D or 3D spiral search
        // spiral_search_2d(&measuredState, &desiredState);  // Call for 2D search
        // Or:
        // spiral_search_3d(&measuredState, &desiredState);  // Call for 3D search with altitude oscillation
        desiredState.yaw = M_PI/4;
        desiredState.x = 0.5;
        desiredState.y = 0.5;
        // if(fabs(measuredState.x - desiredState.x) < 0.01 && fabs(measuredState.x - previousState.x) < 0.005)
        // {
        //   state = LAND;
        //   printf("########################## \n");
        //   printf("LANDING \n");
        //   printf("########################## \n");
        // }
        break;

      case ENGAGE:
        // Engagement logic, if applicable
        if (fabs(measuredState.x - opticalPoint.x + 0.3) < 0.1 && fabs(measuredState.y - opticalPoint.y) < 0.1 && fabs(measuredState.altitude - opticalPoint.z) < 0.1)
        {
          printf("Reached end of path \n");
          state = LAND;
          break;
        }
        else if (fabs(measuredState.x - path[node_counter]->coord.x) < 0.1 && fabs(measuredState.y - path[node_counter]->coord.y) < 0.1 && fabs(measuredState.altitude - path[node_counter]->coord.z) < 0.1)
        {
            printf("Reached node %d\n", node_counter + 1);
            if (node_counter < pathLength - 1)
            {
              node_counter++;
            }
        }
        desiredState.x = path[node_counter]->coord.x;
        desiredState.y = -path[node_counter]->coord.y;
        desiredState.altitude = path[node_counter]->coord.z;
        desiredState.yaw = path[node_counter]->coord.yaw;
        break;

      case LAND:
        desiredState.altitude = 0.1;
        if(fabs(measuredState.altitude - desiredState.altitude) < 0.01 && fabs(measuredState.altitude - prev_altitude) < 0.005)
        {
          state = MISSION_COMPLETE;
          printf("########################## \n");
          printf("Shutting off engines \n");
          printf("########################## \n");
          motorPower.frontLeft = 0;
          motorPower.frontRight = 0;
          motorPower.rearLeft = 0;
          motorPower.rearRight = 0;
        }
        break;

      case MISSION_COMPLETE:
        motorPower.frontLeft = 0;
        motorPower.frontRight = 0;
        motorPower.rearLeft = 0;
        motorPower.rearRight = 0;
    }
      
    // PID velocity controller with fixed height
    // pid_velocity_fixed_height_controller(&measuredState, &desiredState, pidParams, dt, &motorPower);
    
    // positionToAttitude(&desiredState, &measuredState);

    // Calculate control errors
    altitudeError.error = desiredState.altitude - measuredState.altitude;
    vxError.error = (desiredState.x - measuredState.x)/10;
    vyError.error = (desiredState.y - measuredState.y)/10;

    desiredState.roll = -pid(&vyError, dt);
    desiredState.pitch = pid(&vxError, dt);

    rollError.error = constrain(desiredState.roll - measuredState.roll, -1, 1);
    pitchError.error = -constrain(desiredState.pitch - measuredState.pitch, -2, 2);
    yawError.error = desiredState.yaw - measuredState.yaw;

    control_t controlCommands = 
    {
      .altitude = pid(&altitudeError, dt) + 48,
      .roll = pid(&rollError, dt),
      .pitch = pid(&pitchError, dt),
      .yaw = pid(&yawError, dt)
    };
    // printf("Altitude error: %f\n", altitudeError.error);
    // printf("Actual error: %f\n", desiredState.altitude - measuredState.altitude);
    // printf("Control signal, altitude:%f\n", controlCommands.altitude);
    // printf("Control signal, roll:%f\n", controlCommands.roll);
    // printf("Control signal, pitch:%f\n", controlCommands.pitch);
    // printf("Control signal, yaw:%f\n", controlCommands.yaw);
    // printf(" \n");

    if (state != MISSION_COMPLETE)
    {
      controller(&controlCommands, &motorPower);
    }
    

    // Setting motorspeed
    wb_motor_set_velocity(motors.frontLeft, -motorPower.frontLeft);
    wb_motor_set_velocity(motors.rearLeft, motorPower.rearLeft);
    wb_motor_set_velocity(motors.rearRight, -motorPower.rearRight);
    wb_motor_set_velocity(motors.frontRight, motorPower.frontRight);

    // Save past time for next time step
    pastTime = wb_robot_get_time();
    prev_altitude = measuredState.altitude;
    previousState.x = measuredState.x; // Update previous state
    previousState.y = measuredState.y; 
  };

  // Free allocated memory
  for (int i = 0; i < pathLength; i++)
  {
    free(path[i]);
  }

  for (int i = 0; i < numObstacles; i++)
  {
    free(obstacles[i]);
  }

  wb_robot_cleanup();

  return 0;
}

      // if (calc_distance(groundTruth, opticalPoint) < 0.1)
      // {
      //   printf("Reached end of path \n");
      //   sideways_desired = 0;
      //   forward_desired = 0;
      //   height_diff_desired = 0;
      // }
      // else if (calc_distance(groundTruth, path[node_counter]->coord) < 0.1)
      // {
      //     printf("Reached node %d\n", node_counter + 1);
      //     if (node_counter < pathLength - 1)
      //     {
      //       node_counter++;
      //     }
      //     // Update desired state to the next waypoint
      //     sideways_desired = 0.5*(path[node_counter]->coord.y - groundTruth.y);
      //     forward_desired = 0.5*(path[node_counter]->coord.x - groundTruth.x);
      //     height_diff_desired = 0.1*(path[node_counter]->coord.z - groundTruth.z);
      // }