#include "spiral_search.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void spiral_search(cf_state_t *state, float r_step, float yaw_rate)
{
  float angle = state->yaw;
  float radius = state->r;

  // Increase the spiral radius and yaw angle.
  printf("actual radius: %f \n",radius);

  if (radius < state->r_max)
  {
    radius += r_step;
  }
  angle += yaw_rate;

  // Set the desired x and y speeds
  state->vx = radius;

  // Update the relative position
  state->x = state->vx;
  state->y = state->vy;

  // Update the radius and yaw

  state->r = radius;
  state->yaw = angle;
}

void pid_controller(cf_state_t* measured_state, cf_state_t* desired_state,
                    gainsPid_t gains_pid, float dt, motorPower_t* motorCommands)
{
  controlCommands_t control_commands = {0};

  float pastAltitudeError, pastPitchError, pastRollError, pastYawRateError;
  float pastVxError, pastVyError;
  float altitudeIntegrator;

  ///////////////////////////////////////////////////

  float vxError = desired_state->vx - measured_state->vx;
  float vxDerivative = (vxError - pastVxError) / dt;
  float vyError = desired_state->vy - measured_state->vy;
  float vyDerivative = (vyError - pastVyError) / dt;

  // PID control
  float pitchCommand = gains_pid.kp_vel_xy * constrain(vxError, -1, 1) + gains_pid.kd_vel_xy * vxDerivative;
  float rollCommand = -gains_pid.kp_vel_xy * constrain(vyError, -1, 1) - gains_pid.kd_vel_xy * vyDerivative;

  desired_state->pitch = pitchCommand;
  desired_state->roll = rollCommand;

  // Save error for the next round
  pastVxError = vxError;
  pastVyError = vyError;

  /////////////////////////////////////////////////////////////////

  float altitudeError = desired_state->altitude - measured_state->altitude;
  float altitudeDerivativeError = (altitudeError - pastAltitudeError) / dt;
  control_commands.altitude =
    gains_pid.kp_z * constrain(altitudeError, -1, 1) + gains_pid.kd_z * altitudeDerivativeError + gains_pid.ki_z;

  altitudeIntegrator += altitudeError * dt;
  control_commands.altitude = gains_pid.kp_z * constrain(altitudeError, -1, 1) + gains_pid.kd_z * altitudeDerivativeError +
                               gains_pid.ki_z * altitudeIntegrator + 48;
  pastAltitudeError = altitudeError;

  ///////////////////////////////////////////////////////////////////////

  // Calculate errors
  float pitchError = desired_state->pitch - measured_state->pitch;
  float pitchDerivativeError = (pitchError - pastPitchError) / dt;
  float rollError = desired_state->roll - measured_state->roll;
  float rollDerivativeError = (rollError - pastRollError) / dt;
  float yawRateError = desired_state->yaw_rate - measured_state->yaw_rate;

  // PID control
  control_commands.roll = gains_pid.kp_att_rp * constrain(rollError, -1, 1) + gains_pid.kd_att_rp * rollDerivativeError;
  control_commands.pitch = -gains_pid.kp_att_rp * constrain(pitchError, -1, 1) - gains_pid.kd_att_rp * pitchDerivativeError;
  control_commands.yaw = gains_pid.kp_att_y * constrain(yawRateError, -1, 1);

  // Save error for the next round
  pastPitchError = pitchError;
  pastRollError = rollError;
  pastYawRateError = yawRateError;

  /////////////////////////////////////////////////////////////////////////

  // Motor mixing
  motorCommands->m1 = control_commands.altitude - control_commands.roll + control_commands.pitch + control_commands.yaw;
  motorCommands->m2 = control_commands.altitude - control_commands.roll - control_commands.pitch - control_commands.yaw;
  motorCommands->m3 = control_commands.altitude + control_commands.roll - control_commands.pitch + control_commands.yaw;
  motorCommands->m4 = control_commands.altitude + control_commands.roll + control_commands.pitch - control_commands.yaw;

}

float calc_distance(coord_t point_1, coord_t point_2)
{
  return sqrt(pow(point_1.x - point_2.x,2) + pow(point_1.y - point_2.y,2) + pow(point_1.z - point_2.z,2));
}

void engage(cf_state_t* start, float goal)
{
  
}