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
#include <Python.h>

// Add external controller
// #include "pid_controller.h"
#include "spiral_search.h"
#include "path_planner.h"

#define MAX_RADIUS 1.0



void initPython()
{
    // Initialize the Python Interpreter
    Py_Initialize();

    // Load the Python module
    PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;

    // Step 1: Load the Python script (yolov8_integration.py)
    pName = PyUnicode_DecodeFSDefault("yolov8_integration");
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL)
    {
        // Step 2: Get the Python function that processes the image and returns coordinates
        pFunc = PyObject_GetAttrString(pModule, "detect_optical_source");

        // Ensure the function is callable
        if (pFunc && PyCallable_Check(pFunc))
        {
            // Step 3: Call the Python function
            pValue = PyObject_CallObject(pFunc, NULL);

            if (pValue != NULL)
            {
                // Parse the returned Python value (tuple with x and y coordinates)
                double x, y;
                PyArg_ParseTuple(pValue, "dd", &x, &y);

                // Print or use the coordinates in your C program
                printf("Optical Source Coordinates: x = %f, y = %f\n", x, y);
                Py_DECREF(pValue);
            }
            else
            {
                PyErr_Print();
            }
        }
        else
        {
            if (PyErr_Occurred()) PyErr_Print();
        }

        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else
    {
        PyErr_Print();
    }

    // Finalize the Python Interpreter
    Py_Finalize();
}

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
  measuredState->altitude = wb_distance_sensor_get_value(sensors->tofsensor);
  measuredState->altitude *= cos(measuredState->pitch)*cos(measuredState->roll);

  // The Flow deck measures horizontal velocity through optical flow
  // Due to no pre-made opticalflow sensors this is simulated through
  // GPS measurements

  float pos_x = wb_gps_get_values(sensors->gps)[0];
  float pos_y = wb_gps_get_values(sensors->gps)[1];

  measuredState->vx = (pos_x - previousState->x) / dt;
  measuredState->vy = (pos_y - previousState->y) / dt;
  velocityTransform(&measuredState);

  measuredState->x += measuredState->vx*dt; // Position is (re)calculated
  measuredState->y += measuredState->vy*dt; // using velocity.
}

int main(int argc, char **argv) {
  wb_robot_init();
  initPython();

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
  int state = START;

  coord_t opticalPoint = {2.15,0.3,1.5}; // Location of the optical point in global coordinates

  // Initialize camera variables
  // bool autonomous = false;
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
  int nodeCounter = 1; // Keeps count of which node in the path the MAV is aiming for during ENGAGE state

  // Wait for 2 seconds
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }
  
  // Initialize state variables
  cf_state_t measuredState = {0};
  cf_state_t desiredState = {.altitude = FLYING_ALTITUDE};
  cf_state_t previousState = {0};

  coord_t spiralCenter = {0}; // Keep track of the center of the search pattern

  // Initialize PID gains.
  controlError_t vxError = {.kp = 2, .ki = 0, .kd = 0.5, .integrator = 0, .previousError = 0};
  controlError_t vyError = {.kp = -2, .ki = 0, .kd = -0.5, .integrator = 0, .previousError = 0};
  controlError_t altitudeError = {.kp = 10, .ki = 1.5, .kd = 20, .integrator = 0, .previousError = 0}; // Well tuned
  controlError_t rollError = {.kp = 0.5, .ki = 0, .kd = 0.1, .integrator = 0, .previousError = 0};
  controlError_t pitchError = {.kp = -0.5, .ki = 0, .kd = -0.1, .integrator = 0, .previousError = 0};
  controlError_t yawError = {.kp = 2, .ki = 0, .kd = 1, .integrator = 0, .previousError = 0}; // Well tuned

  float pastTime = wb_robot_get_time();

  // Initialize struct for motor power
  motorPower_t motorPower;

  // Starting and goal positions (x, y, z, yaw)
  coord_t start = {measuredState.x, measuredState.y, FLYING_ALTITUDE, measuredState.yaw};
  coord_t goal = {opticalPoint.x - 0.3, opticalPoint.y, opticalPoint.z, 0}; // Goal position
  
  float totalCost = AStar(start, goal, path, &pathLength, &obstacles, numObstacles);
  printPath(path, pathLength);
  // drawPath(path, pathLength, obstacles, numObstacles);

  while (wb_robot_step(timestep) != -1)
  {
    const float dt = wb_robot_get_time() - pastTime;

    dt_counter += dt; // Sums up time passage for camera recording frame rate calculations
    
    // Collect sensor measurements |    
    measurementUpdate(&sensors, &measuredState, &previousState, dt);

    // Check for keyboard input
    int key = wb_keyboard_get_key();
    while (key > 0)
    {
      switch (key)
      {
        // Disable autonomous mode
        case 'D': 
          printf("---Returning to manual control---\n");
          // autonomous = false;
          break;

        // Simulate network activation, i.e. optical point found
        case 'N':
          if(!network)
          {
            network = true;
            printf("---Light source detected!---\n");

            state = ENGAGE;
            nodeCounter = 0;
            pathLength = 0;
            
            // Starting position (x, y, z, yaw)
            coord_t start = {measuredState.x, measuredState.y, measuredState.z, measuredState.yaw*dt*180/M_PI};
            coord_t goal = {opticalPoint.x - 0.3, opticalPoint.y, opticalPoint.z, 0}; // Goal position
            
            float totalCost = AStar(start, goal, path, &pathLength, &obstacles, numObstacles);
            printPath(path, pathLength);
          }
          break;
        
        // Simulate network deactivation, i.e. optical point lost
        case 'L':
          if(network)
          {
            printf("---Light source lost. Returning to search---\n");
            network = false;
            spiralCenter.x = measuredState.x;
            spiralCenter.y = measuredState.y;
            spiralCenter.z = measuredState.altitude;
          }
          break;

        // 
        case 'O':
          printf("---Light source lost. Returning to search---\n");
          Obstacle obstacle = // Define an obstacle
          {
            .min={0.6, -1.2, 0},
            .max={1.0, 0.4, 4},
            .margin=0.3
          };
          obstacles[0] = &obstacle;
          numObstacles = 1;
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

    //  |------------------------------------------------------------|
    //  |-------------------- STATE  MACHINE ------------------------|
    //  |------------------------------------------------------------|
    switch (state)
    {
      case START:
        if(fabs(measuredState.altitude - FLYING_ALTITUDE) < 0.01 && fabs(measuredState.altitude - previousState.altitude) < 0.005)
        {
          spiralCenter.x = measuredState.x;
          spiralCenter.y = measuredState.y;
          spiralCenter.z = measuredState.altitude;
          state = SEARCH;
        }
        break;

      case SEARCH:  // TODO: Add spiral search logic here
        spiralSearch(&measuredState, &desiredState, &spiralCenter);
        break;

      case ENGAGE:
        if (pathLength == 0)
        {
          break;
        }
        if (fabs(measuredState.x - goal.x) < 0.1 && fabs(measuredState.y - goal.y) < 0.1 && fabs(measuredState.altitude - goal.z) < 0.1)
        {
          printf("Reached end of path \n");
          // Starting and goal positions (x, y, z, yaw)'
          nodeCounter = 0;
          // coord_t start = {measuredState.x, measuredState.y, FLYING_ALTITUDE, measuredState.yaw};
          // coord_t goal = {0.5, 0.5, 0.5, -M_PI}; // Goal position
          
          // float totalCost = AStar(start, goal, path, &pathLength, &obstacles, numObstacles);
          // printPath(path, pathLength);
          break;
        }
        else if (fabs(measuredState.x - path[nodeCounter]->coord.x) < 0.1 && fabs(measuredState.y - path[nodeCounter]->coord.y) < 0.1 && fabs(measuredState.altitude - path[nodeCounter]->coord.z) < 0.1)
        {
            printf("Reached node %d\n", nodeCounter + 1);
            printf("Measured state: (%f, %f, %f, %f)\n", measuredState.x, measuredState.y, measuredState.altitude, measuredState.yaw*180/M_PI);
            printf(" \n");
            if (nodeCounter < pathLength - 1)
            {
              nodeCounter++;
            }
        }
        desiredState.x = path[nodeCounter]->coord.x;
        desiredState.y =  path[nodeCounter]->coord.y;
        desiredState.altitude = path[nodeCounter]->coord.z;
        desiredState.yaw = path[nodeCounter]->coord.yaw;
        break;

      case LAND:
        desiredState.altitude = 0.1;
        if(measuredState.altitude < 0.1 && fabs(measuredState.altitude - previousState.altitude) < 0.005)
        {
          state = MISSION_COMPLETE;
          printf("########################## \n");
          printf("MISSION COMPLETE \n");
          printf("########################## \n");
        }
        break;

      case MISSION_COMPLETE:
        motorPower.frontLeft = 0;
        motorPower.frontRight = 0;
        motorPower.rearLeft = 0;
        motorPower.rearRight = 0;
    }
       
    //  Calculate control error
    altitudeError.error = desiredState.altitude - measuredState.altitude;
    vxError.error = (desiredState.x - measuredState.x)/10;
    vyError.error = (desiredState.y - measuredState.y)/10;

    desiredState.roll = pid(&vyError, dt);
    desiredState.pitch = pid(&vxError, dt);

    rollError.error = constrain(desiredState.roll - measuredState.roll, -1, 1);
    pitchError.error = constrain(desiredState.pitch - measuredState.pitch, -2, 2);
    yawError.error = desiredState.yaw - measuredState.yaw;

    control_t controlCommands = 
    {
      .altitude = pid(&altitudeError, dt) + 48, // ASCII??? Lyfter ej utan + 48...
      .roll = pid(&rollError, dt),
      .pitch = pid(&pitchError, dt),
      .yaw = pid(&yawError, dt)
    };

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
    previousState = measuredState; // Update previous state
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