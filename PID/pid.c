#include "pid.h"

void pid_init_state(pid_state_t* state, float Kp, float Ki, float Kd)
{
  state->Kp = Kp;
  state->Ki = Ki;
  state->Kd = Kd;

  state->last_error = 0;
  state->integral = 0;
  state->target = 0;
}

void pid_set_target(pid_state_t* state, int target)
{
  state->target = target;
  state->integral = 0;
  state->last_error = 0;
}

int pid(pid_state_t* state, int pos, int dt)
{
  int error = state->target - pos;

  state->integral += (error * dt)/1000;

  int derivative = state->last_error - error;

  int output = state->Kp * error + state->Ki * state->integral + state->Kd * derivative;

  // constrict output to -127 to 127
  if(output > 127) output = 127;
  if(output < -127) output = -127;

  // clean up
  state->last_error = error;

  return output;
}
