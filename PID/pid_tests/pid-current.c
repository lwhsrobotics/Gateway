typedef struct
{
  float Kp;
  float Ki;
  float Kd;

  int lastError;

  float integral;

  int target;
} PIDState;


/*************************************************
 * call *right* after creating a PIDState object *
 * note that K variables still need to be set    *
 *************************************************/
void pid_init_state(PIDState* state)
{
  state->Kp = 0;
  state->Ki = 0;
  state->Kd = 0;

  state->lastError = 0;
  state->integral = 0;
  state->target = 0;
}


/**********************************
 * calculate new PID value        *
 * elapsedTime is in milliseconds *
 **********************************/
int pid(PIDState* state, int position, int elapsedTime)
{
  int error = state->target - position;

  state->integral += error * elapsedTime;

  int derivative = state->lastError - error;

  int output = state->Kp * error + state->Ki * state->integral + state->Kd * derivative;

  // constrict output to -127 to 127
  if(output > 127) output = 127;
  if(output < -127) output = -127;

  // clean up
  state->lastError = error;

  return output;
}

/*
This is Jacob's original code:

typedef struct
{
  float dState;      // last position input
  float iState;      // integral state
  float iMax, iMin;

  float iGain, // integral constant
        pGain, // proportional constant
        dGain; // derivative constant
} SPid;

int updatePID(SPid * pid, float error, float elapsedTime)
{
  float pTerm, dTerm, iTerm;

  pTerm = pid->pGain * error;

  pid->iState += error;

  if (pid->iState > pid->iMax) pid->iState = pid->iMax;
  else if (pid-> iState < pid->iMin) pid->iState = pid->iMin;

  iTerm = pid->iGain * pid->iState;

  dTerm = pid->dGain * (position - pid->dState);

  pid->dState = position;

  return pTerm + iTerm - dTerm;
}*/
