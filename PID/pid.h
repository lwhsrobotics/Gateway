#ifndef PID_H
#define PID_H

/*******************************************************************************
 * Structure to contain information for PID.                                   *
 *                                                                             *
 * By default this will create an invalid structure.  Run pid_init_state on it *
 * before using the structure.                                                 *
 *******************************************************************************/
typedef struct
{
  float Kp;
  float Ki;
  float Kd;

  int last_error;

  float integral;

  int target;
} pid_state_t;

/*********************************************************
 * Initialize the PID state using the three K constants. *
 *********************************************************/
void pid_init_state(pid_state_t* state, float Kp, float Ki, float Kd);

/*****************************
 * Set the PID target value. *
 *****************************/
void pid_set_target(pid_state_t* state, int target);

/***************************************************
 * Calculate PID.                                  *
 *                                                 *
 * pos: Real position to calculate error from.     *
 * dt: Time in milliseconds since the last update. *
 ***************************************************/
int pid(pid_state_t* state, int pos, int dt);

#endif // PID_H
