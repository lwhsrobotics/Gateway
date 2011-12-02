#pragma config(Motor,  port2,           driveLeft,     tmotorNormal, openLoop)
#pragma config(Motor,  port3,           driveRight,    tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port5,           liftLeft,      tmotorNormal, openLoop)
#pragma config(Motor,  port6,           liftRight,     tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port8,           claw,          tmotorNormal, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

// Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"

/* Define Constants */
#define ON 1
#define MAX_SPEED 127
#define MIN_SPEED -127
#define CLAW_CLOSED_POS -90
#define CLAW_OPENED_POS 0

/* Forward Declarations */
void drive(int x, int y);
void lift(int x);
void toggleClawState();

/* Global Variables */
typedef enum
{
  CLAW_OPENED,
  CLAW_CLOSED
} ClawState;

bool clawButtonPressed;
ClawState clawState;

void pre_auton()
{
	// initialize global variables
	clawButtonPressed = false;
	clawState = CLAW_OPENED;
}

/* 20 Second Autonomous Period */
task autonomous()
{
	drive(0, MAX_SPEED);
	wait1Msec(1500);
	drive(0, 0);
	wait1Msec(1000);
	drive(0, MIN_SPEED);
	wait1Msec(1500);
	drive(0, 0);
}

/* User Controlled Period */
task usercontrol()
{
	// joystick layout:
	//   left joystick: arm control
	//   right joystick: drive control
	//   right rear upper bumper: claw toggle

	while (true)
	{
		// drive according to the right joystick
		drive(vexRT[Ch1], vexRT[Ch2]);

		// lift according to the left joystick
		lift(vexRT[Ch3]);

		// open and close the claw
		if (vexRT[Btn6U] == ON)
		{
			if(!clawButtonPressed)
			{
				// claw button just now pressed
				toggleClawState();
			}

			clawButtonPressed = true;
		}
		else
		{
			clawButtonPressed = false;
		}
	}
}

// smooths speed using x^2 curve
int smoothSpeed(int speed)
{
	return speed * abs(speed) / MAX_SPEED;
}

// x = x axis value
// y = y axis value
void drive(int x, int y)
{
	// drive function: f(x) = (x,x) <-- (left motor, right motor)
	// turn function: f(x) = (x,-x) <-- (left motor, right motor)

	// prevent divide-by-zero
  if((x + y) == 0)
  {
    motor[driveLeft] = 0;
    motor[driveRight] = 0;
  }
  else
  {
  	motor[driveLeft] = smoothSpeed((x * x + y * y) / (x + y));
	  motor[driveRight] = smoothSpeed(-x + y);
  }
	//motor[driveRight] = (-x*abs(x) + y*abs(y)) / (x + y);
}

// set lift motors using smoothed x
void lift(int x)
{
	motor[liftLeft] = x;
	motor[liftRight] = x;
}

void toggleClawState()
{
	// clawState is a global variable
	if(clawState == CLAW_CLOSED) // if claw was closed, open it
	{
		motor[claw] = CLAW_OPENED_POS;
		clawState = CLAW_OPENED;
	}
	else // if claw was open, close it
	{
		motor[claw] = CLAW_CLOSED_POS;
		clawState = CLAW_CLOSED;
	}
}
