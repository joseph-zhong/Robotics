#pragma config(Sensor, in2,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl5,  rightEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  leftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl9,  RightButton,          sensorTouch)
#pragma config(Sensor, dgtl10, LeftButton,          sensorTouch)
#pragma config(Sensor, dgtl11, Auto3,          sensorTouch)
#pragma config(Motor,  port1,           Intake1,       tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port2,           Drive1,        tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port3,           Drive2,        tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port4,           Drive3,        tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port5,           Drive4,        tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port6,           Drive5,        tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port7,           Drive6,        tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port8,           Arm1,          tmotorVex393, openLoop)
#pragma config(Motor,  port9,           Arm2,          tmotorVex393, openLoop)
#pragma config(Motor,  port10,          Intake2,       tmotorVex393HighSpeed, openLoop)
//This code is for 917S//
//These are the configuration statements

#pragma platform(VEX)

#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"

#define FORWARD 127
#define BACKWARD -127
#define STOP 0
//If you don't want to use the motor values of 127 -127 or 0,you can use these instead so that the code makes more sense

int encoders=((SensorValue[rightEncoder]+SensorValue[leftEncoder])/2);
int encoderVal=((SensorValue[rightEncoder]+SensorValue[leftEncoder])/2);

void travelToRampedHalt(int maxPwr, int distTotal, int rampDist)
{
 //zero encoders
 int minPwr=50;//this should be the minimum power level required for reliable forward motion
 //set motors to maxPwr

 while(encoders < distTotal) //ramp down time
 {
    int pwr = maxPwr - (maxPwr - minPwr)*(encoderVal - distTotal + rampDist)/rampDist; //this scales the power from max->minPwr over the rampDist

    //set motors to pwr
 }
 //set motors to zero
}
//I did not use this function because we did not have time to practice with it
//Mid is the one that goes straight to the stash and stashes one ball, we will not use this and I will disable it
void Mid ()
{
		motor[Intake1]=-127;
		motor[Intake2]=-127;
		wait1Msec(250);
		motor[Intake1]=0;
		motor[Intake2]=0;
		//flips down intake by outtaking for .25 seconds and then stopping the intake so that it does not outtake the ball
		wait1Msec(500);
		//time for last second repositioning
		motor[Drive1]=100;
		motor[Drive2]=100;
		motor[Drive3]=100;
		motor[Drive4]=100;
		motor[Drive5]=100;
		motor[Drive6]=100;
		wait1Msec(1200);
		//drive near stash
		motor[Arm1]=127;
		motor[Arm2]=127;
		wait1Msec(1);
		//raises arm to full capacity
		motor[Arm1]=0;
		motor[Arm2]=0;
		motor[Drive1]=50;
		motor[Drive2]=50;
		motor[Drive3]=50;
		motor[Drive4]=50;
		motor[Drive5]=50;
		motor[Drive6]=50;
		wait1Msec(.5);
		// stops arm and drives the robot all the way to the stash, this also centers our robot around the stash
		motor[Drive1]=0;
		motor[Drive2]=0;
		motor[Drive3]=0;
		motor[Drive4]=0;
		motor[Drive5]=0;
		motor[Drive6]=0;
		motor[Intake1]=-127;
		motor[Intake2]=-127;
		wait1Msec(1);
		//drive stops and the intake outtakes
		motor[Intake1]=motor[Intake2]=0;
		//stop intaking

}
//Mid2 is the 15pt middle auton in which we knock both balls off and stash one bucky
void Mid2 ()
{
		motor[Intake1]=-127;
		motor[Intake2]=-127;
		wait1Msec(250);
		motor[Intake1]=0;
		motor[Intake2]=0;
		//flip down intake
		motor[Arm1]=127;
    motor[Arm2]=127;
		wait1Msec(400);
		motor[Arm1]=10;
		motor[Arm2]=10;
		//raise arm to barrier level
		wait1Msec(1000);
		motor[Drive1]=127;
		motor[Drive2]=127;
		motor[Drive3]=127;
		motor[Drive4]=127;
		motor[Drive5]=127;
		motor[Drive6]=127;
		wait1Msec(800);
		//hit closest ball
		motor[Drive1]=0;
		motor[Drive2]=0;
		motor[Drive3]=0;
		motor[Drive4]=0;
		motor[Drive5]=0;
		motor[Drive6]=0;
		wait1Msec(1000);
		// stop drive, if we were to drive straight back after stopping, we might risk tipping
		motor[Drive1]=-127;
		motor[Drive2]=-127;
		motor[Drive3]=-127;
		motor[Drive4]=-127;
		motor[Drive5]=-127;
		motor[Drive6]=-127;
		wait1Msec(800);
		//drive back to tile
		motor[Drive1]=0;
		motor[Drive2]=0;
		motor[Drive3]=0;
		motor[Drive4]=0;
		motor[Drive5]=0;
		motor[Drive6]=0;
		wait1Msec(1000);
		//lets alex reposition the robot and stops drive
		motor[Drive1]=127;
		motor[Drive2]=127;
		motor[Drive3]=127;
		motor[Drive4]=127;
		motor[Drive5]=127;
		motor[Drive6]=127;
		wait1Msec(1000);
		//drives to the second ball
		motor[Drive1]=0;
		motor[Drive2]=0;
		motor[Drive3]=0;
		motor[Drive4]=0;
		motor[Drive5]=0;
		motor[Drive6]=0;
		wait1Msec(1000);
		//drive stops
		motor[Drive1]=-127;
		motor[Drive2]=-127;
		motor[Drive3]=-127;
		motor[Drive4]=-127;
		motor[Drive5]=-127;
		motor[Drive6]=-127;
		wait1Msec(450);
		//drives back for .45 before lowering arm so that the intake and arm don't catch the barrier when lowering.
		motor[Arm1]=motor[Arm2]=-127;
		wait1Msec(500);
		motor[Drive1]=0;
		motor[Drive2]=0;
		motor[Drive3]=0;
		motor[Drive4]=0;
		motor[Drive5]=0;
		motor[Drive6]=0;
		motor[Arm1]=motor[Arm2]=0;
		wait1Msec(1000);
		//alex repositions toward stash
		motor[Drive1]=100;
		motor[Drive2]=100;
		motor[Drive3]=100;
		motor[Drive4]=100;
		motor[Drive5]=100;
		motor[Drive6]=100;
		wait1Msec(1250);
		//to stash
		motor[Drive1]=0;
		motor[Drive2]=0;
		motor[Drive3]=0;
		motor[Drive4]=0;
		motor[Drive5]=0;
		motor[Drive6]=0;
		//stop drive
		motor[Arm1]=127;
		motor[Arm2]=127;
		wait1Msec(1000);
		//raise arm
		motor[Arm1]=0;
		motor[Arm2]=0;
		motor[Drive1]=50;
		motor[Drive2]=50;
		motor[Drive3]=50;
		motor[Drive4]=50;
		motor[Drive5]=50;
		motor[Drive6]=50;
		wait1Msec(500);
		// drive to stash and center around it
		motor[Drive1]=0;
		motor[Drive2]=0;
		motor[Drive3]=0;
		motor[Drive4]=0;
		motor[Drive5]=0;
		motor[Drive6]=0;
		//stop drive
		motor[Intake1]=-127;
		motor[Intake2]=-127;
		wait1Msec(1000);
		//outtake
		motor[Intake1]=motor[Intake2]=0;
		//stop intake

}

void Mid3()
{
	motor[Intake1]=-127;
	motor[Intake2]=-127;
	wait1Msec(250);
	motor[Intake1]=0;
	motor[Intake2]=0;
	// flips out intake
	motor[Drive1]=motor[Drive2]=motor[Drive3]=motor[Drive4]=motor[Drive5]=motor[Drive6]=100;
	motor[Intake1]=motor[Intake2]=127;
	wait1Msec(1000);
	//goes over the bump while intaking and picks up the two buckies against wall
	motor[Drive1]=motor[Drive2]=motor[Drive3]=motor[Drive4]=motor[Drive5]=motor[Drive6]=0;
	wait1Msec(500);
	//stops drive
	motor[Drive1]=motor[Drive2]=motor[Drive3]=motor[Drive4]=motor[Drive5]=motor[Drive6]=-100;
	wait1Msec(1000);
	//returns to tile
	motor[Drive1]=motor[Drive2]=motor[Drive3]=motor[Drive4]=motor[Drive5]=motor[Drive6]=0;
	wait1Msec(2000);
	//stops drive, gives alex two seconds to turn the robot 180 degrees
	motor[Intake1]=0;
	motor[Intake2]=0;
	//stops intake because, if it attempts to intake large ball on barrier, then the intake will flip back up
	motor[Arm1]=100;
  motor[Arm2]=100;
	wait1Msec(475);
	//raise arm to barrier level
	motor[Arm1]=0;
	motor[Arm2]=0;
	wait1Msec(1000);
	//stop arm
	motor[Drive1]=127;
	motor[Drive2]=127;
	motor[Drive3]=127;
	motor[Drive4]=127;
	motor[Drive5]=127;
	motor[Drive6]=127;
	wait1Msec(800);
	//hit closest large ball off barrier
	motor[Drive1]=0;
	motor[Drive2]=0;
	motor[Drive3]=0;
	motor[Drive4]=0;
	motor[Drive5]=0;
	motor[Drive6]=0;
	wait1Msec(1000);
	//stops drive
	motor[Drive1]=-127;
	motor[Drive2]=-127;
	motor[Drive3]=-127;
	motor[Drive4]=-127;
	motor[Drive5]=-127;
	motor[Drive6]=-127;
	wait1Msec(800);
	//returns to tile
	motor[Drive1]=0;
	motor[Drive2]=0;
	motor[Drive3]=0;
	motor[Drive4]=0;
	motor[Drive5]=0;
	motor[Drive6]=0;
	wait1Msec(1000);
	//allows for alex to reposition towards the stash
	motor[Drive1]=100;
	motor[Drive2]=100;
	motor[Drive3]=100;
	motor[Drive4]=100;
	motor[Drive5]=100;
	motor[Drive6]=100;
	wait1Msec(1200);
	//to stash
	motor[Drive1]=0;
	motor[Drive2]=0;
	motor[Drive3]=0;
	motor[Drive4]=0;
	motor[Drive5]=0;
	motor[Drive6]=0;
	//stop drive
	motor[Arm1]=127;
	motor[Arm2]=127;
	wait1Msec(1200);
	//raise arm
	motor[Arm1]=0;
	motor[Arm2]=0;
	motor[Drive1]=50;
	motor[Drive2]=50;
	motor[Drive3]=50;
	motor[Drive4]=50;
	motor[Drive5]=50;
	motor[Drive6]=50;
	wait1Msec(500);
	// drive to stash and center around it
	motor[Drive1]=0;
	motor[Drive2]=0;
	motor[Drive3]=0;
	motor[Drive4]=0;
	motor[Drive5]=0;
	motor[Drive6]=0;
	//stop drive
	motor[Intake1]=-127;
	motor[Intake2]=-127;
	wait1Msec(1000);
	//outtake
	motor[Intake1]=motor[Intake2]=0;
	//stop intake
}
//the ol'rammarooooo
void Hang()
{
	motor[Intake1]=-127;
	motor[Intake2]=-127;
	wait1Msec(250);
	motor[Intake1]=0;
	motor[Intake2]=0;
	motor[Drive1]=motor[Drive2]=motor[Drive3]=motor[Drive4]=motor[Drive5]=motor[Drive6]=127;
	wait1Msec(2000);
	motor[Drive1]=motor[Drive2]=motor[Drive3]=motor[Drive4]=motor[Drive5]=motor[Drive6]=0;

}
void pre_auton()
{
	SensorValue(rightEncoder)=0;
	SensorValue(leftEncoder)=0;
}

task autonomous()
{
	while(1)
		if(SensorBoolean[RightButton])
			Mid2();
		else if(SensorBoolean[LeftButton])
			Mid3();
		else if(SensorBoolean[Auto3])
			Hang();
	//nothing -1814 (right), 1727 (left)
			//so basicarryyy, if one of the touch sensors returns the boolean 1(true) then you run it
}

task usercontrol()
{
while (true) {
int HOLD = 20;
	if(vexRT[Btn6U])
		motor[Intake1] = motor[Intake2] = FORWARD;
	else if(vexRT[Btn6D])
		motor[Intake1] = motor[Intake2] = BACKWARD;
	else
		motor[Intake1] = motor[Intake2] = STOP;

	if(vexRT[Btn5U])
		motor[Arm1] = motor[Arm2] = FORWARD;
	else if(vexRT[Btn5D])
		motor[Arm1] = motor[Arm2] = BACKWARD;
	else
		motor[Arm1] = motor[Arm2] = HOLD;
//Right side of the robot is controlled by the right joystick, Y-axis
  motor[Drive2] = vexRT[Ch2];
  motor[Drive3] = vexRT[Ch2];
  motor[Drive4] = vexRT[Ch2];
  //Left side of the robot is controlled by the left joystick, Y-axis
  motor[Drive1] = vexRT[Ch3];
  motor[Drive5] = vexRT[Ch3];
  motor[Drive6] = vexRT[Ch3];


}
}
