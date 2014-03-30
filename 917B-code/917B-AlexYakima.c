#pragma config(Motor,  port1,           frontRightMotor, openLoop, reversed)
#pragma config(Motor,  port2,           midRightMotor, openLoop, reversed)
#pragma config(Motor,  port3,           backRightMotor, openLoop, reversed)
#pragma config(Motor,  port4,           frontLeftMotor, openLoop)
#pragma config(Motor,  port5,           midLeftMotor, tmotorNormal, openLoop)
#pragma config(Motor,  port6,           backLeftMotor, tmotorNormal, openLoop)
#pragma config(Motor,  port7,						armLeftMotor, tmotorNormal, openLoop)
#pragma config(Motor,  port8,						armRightMotor, tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port9,						intakeLeftMotor, tmotorNormal, openLoop)
#pragma config(Motor,  port10,					intakeRightMotor, tmotorNormal, openLoop, reversed)

//turning is set to functions so that one cannot move while turning
//make sure to run the turn functions after the joystick input

//sets a function for a general rotating turn
void Rotate(int magnitude)
{
	motor[frontRightMotor] = -magnitude;
	motor[midRightMotor] = -magnitude;
	motor[backRightMotor] = -magnitude;
	motor[frontLeftMotor] = magnitude;
	motor[midLeftMotor] = magnitude;
	motor[backLeftMotor] = magnitude;
}

void MoveLinear(int magnitude)
{
	motor[frontRightMotor] = magnitude;
	motor[midRightMotor] = magnitude;
	motor[backRightMotor] = magnitude;
	motor[frontLeftMotor] = magnitude;
	motor[midLeftMotor] = magnitude;
	motor[backLeftMotor] = magnitude;
}
void Pivot(int fmagnitude, int smagnitude)
{
	motor[frontRightMotor] = fmagnitude;
	motor[midRightMotor] = fmagnitude;
	motor[backRightMotor] = fmagnitude;
	motor[frontLeftMotor] = fmagnitude;
	motor[midLeftMotor] = fmagnitude;
	motor[backLeftMotor] = fmagnitude;
	if(smagnitude < 0)
	{
		motor[frontLeftMotor] += smagnitude;
		motor[midLeftMotor] += smagnitude;
		motor[backLeftMotor] += smagnitude;
	}
	else
	{
		motor[frontRightMotor] -= fmagnitude;
		motor[midRightMotor] -= fmagnitude;
		motor[backRightMotor] -= fmagnitude;
	}
}

task main()
{
	int leftStickY;
	int rightStickX;
	while(true)
	{
			leftStickY = vexRT[Ch3];
			rightStickX = vexRT[Ch1];


			//left stick y-axis is for moving forwards/backwards
			//right stick x-axis is for turning in place
			//both is for pivot turning
			if(leftStickY != 0 && rightStickX == 0)
			{
				//applied logarithmic scaling
				MoveLinear((leftStickY^2)/127);
			}
			else if(leftStickY == 0 && rightStickX != 0)
			{
				Rotate(rightStickX);
			}
			else if(leftStickY != 0 && rightStickX != 0)
			{
				Pivot(leftStickY, rightStickX);
			}



			//checks for button presses
			//right triggers are for intake; top is in, bottom is out
			//left triggers are for arm; top is up, bottom is down.
			//TODO: check for potentiometer input
			//raises/lowers arms based on button input and holds arms when there is no input
			if(vexRT[Btn6U] == 1)
	    {
	      motor[armLeftMotor] = 40;
	      motor[armRightMotor] = 40;
	    }
	    else if(vexRT[Btn6D] == 1)
	    {
	      motor[armLeftMotor] = -40;
	      motor[armRightMotor] = -40;
	    }
	    else
	    {
	      motor[armLeftMotor] = 0;
	      motor[armRightMotor] = 0;
	    }



	}
}
