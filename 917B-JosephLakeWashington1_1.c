#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    LeftArmAngle,   sensorPotentiometer)
#pragma config(Sensor, in8,    RightArmAngle,  sensorPotentiometer)
#pragma config(Sensor, dgtl12, waitingButton,  sensorTouch)
#pragma config(Sensor, I2C_1,  LeftIEM,        sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  RightIEM,       sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           LeftArm,       tmotorVex393, openLoop)
#pragma config(Motor,  port2,           RightFWheel,   tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port3,           RightMWheel,   tmotorVex393HighSpeed, openLoop, encoder, encoderPort, I2C_2, 1000)
#pragma config(Motor,  port4,           RightBWheel,   tmotorVex393HighSpeed, openLoop, reversed)
#pragma config(Motor,  port5,           LeftIntake,    tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port6,           RightIntake,   tmotorVex393HighSpeed, openLoop, reversed)
#pragma config(Motor,  port7,           LeftBWheel,    tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port8,           LeftMWheel,    tmotorVex393HighSpeed, openLoop, encoder, encoderPort, I2C_1, 1000)
#pragma config(Motor,  port9,           LeftFWheel,    tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port10,          RightArm,      tmotorVex393, openLoop, reversed)

#pragma platform(VEX)
//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"

/******************************************
///////// FINAL VARIABLES HERE ///////////
///////// PLACE NECESSARY USER ///////////
//////////CONTROL AND AUTON //////////////
///////// FUNCTIONS HERE /////////////////
******************************************/

////////////////////////////
//***** Drive Values *****//
////////////////////////////
	int FORWARD = 0;
	int BACKWARD = 1;
	int LEFT = 2;
	int RIGHT = 3;

	int TILE = 684; // Theoretical calculation of 24"
	int HALF_TILE = 342; // 12"
	int TICK_THEORY = 2865; // ticks per hundred inches
	int TICK_AVG = 2850; // ticks per hundred inches
	int PRE_GOAL = 1600;
	int GOAL = 1900;
	int RIGHT_ANGLE = 50; // must test

	int magnitude = 64;
	int control = 15;
////////////////////
//** Arm Values **//
////////////////////
	int BARRIER = 1300;	// Potentiometer value for arm to go over 12" barrier
	int LOW = 580;			// Potentiometer value for arm to reach minimum...Actual value is 550-590...Safety 600 is too high
	int BUMP = 750;			// Lag between pinion and 60 tooth gear -> ranges 600-800...Safety 750
	int HIGH = 1780;		// Ranges between 1750-1900, left is 1750 and right is 1880...Safety 1750
	int PRE_HIGH = 1700;// Just before stretched maximum reach

	int hold = 20; // Arbitrary Numbers tested: 45 too high, 30 holds, 25 holds, 20 holds

/******************************************
///////// VOID FUNCTIONS HERE ////////////
///////// PLACE NECESSARY USER ///////////
//////////CONTROL AND AUTON //////////////
///////// FUNCTIONS HERE /////////////////
******************************************/
	void pre_auton()
	{
		bStopTasksBetweenModes = true; // This is necessary
	}

// Drive Functions here
	int cubicScaling(int x)
	{
	 	return ((((x*3)/25)*((x*3)/25)*((x*3)/25)/27 + x/2)*2)/3; //dead zone of +-3...
	}

	int hardCubicScaling(int x)
	{
		return ((x*x*x) / (127*127)); //dead zone of +- 25...
	}

// Auton Functions here
	int goalTicks(int tenthsOfInches)
	{
	/**************************
time
		600 milliseconds is ~1-1.5 tiles
ticks
		28 ticks undershoot / inch
		29 overshoot / inch
		28.5 average
		28.64791 theory / inch
		2865 / 100 inches
		342 ticks per half tile
		684 ticks per tile
		TICKS/DEGREE TO BE DETERMINED
distance
		1 tile = 24"
		0.5 tile = 12"
theory
		(360 deg / inches circumference) * inches goalDistance = EncoderTicks
	**************************/
		return (TICK_THEORY * tenthsOfInches) / 1000;
	}

	void setLeft(int pwr)
	{
		motor[LeftBWheel] = motor[LeftMWheel] = motor[LeftFWheel] = pwr;
	}

	void setRight(int pwr)
	{
		motor[RightBWheel] = motor[RightMWheel] = motor[RightFWheel] = pwr;
	}

	void resetValues(int wait) //reset values to zero for safety - Encoders, Timers and Motors
	{
			wait1Msec(wait);
			motor[LeftBWheel] = motor[LeftMWheel] = motor[LeftFWheel] = 0;
			motor[RightBWheel] = motor[RightMWheel] = motor[RightFWheel] = 0; // drive motors set to zero
			motor[LeftArm] = motor[RightArm] = 0; // arm motors set to zero
			motor[LeftIntake] = motor[RightIntake] = 0; // intake motors set to zero
			nMotorEncoder[LeftMWheel] = 0; //IEMs set to zero
	}

	void preciseDriveStop(int Case) // For now... Select each case to precision stop, idk how to get motor power
	{
		if(Case == FORWARD) // FORWARD if moving forwards -> negative input sharp stop
			motor[LeftBWheel] = motor[LeftMWheel] = motor[LeftFWheel] = motor[RightBWheel] = motor[RightMWheel] = motor[RightFWheel] = -1;
		else if(Case == BACKWARD) // BACKWARD etc...
			motor[LeftBWheel] = motor[LeftMWheel] = motor[LeftFWheel] = motor[RightBWheel] = motor[RightMWheel] = motor[RightFWheel] = 1;
		else if(Case == LEFT) // LEFT
		{
			motor[LeftBWheel] = motor[LeftMWheel] = motor[LeftFWheel] = 1;
			motor[RightBWheel] = motor[RightMWheel] = motor[RightFWheel] = -1;
		}
		else if(Case == RIGHT) // RIGHT
		{
			motor[LeftBWheel] = motor[LeftMWheel] = motor[LeftFWheel] = -1;
			motor[RightBWheel] = motor[RightMWheel] = motor[RightFWheel] = 1;
		}
		else
			motor[LeftBWheel] = motor[LeftMWheel] = motor[LeftFWheel] = motor[RightBWheel] = motor[RightMWheel] = motor[RightFWheel] = 0;
		wait1Msec(1000); // stabilization time
	}

	// time is in milliseconds
	// distance is in tenths of inches
	// direction is 1 or -1 -- positive is forwards
	void moveStraight(int direction, int time, int driveTarget)
	{
		nMotorEncoder[LeftMWheel] = 0;
		int distance = 0;
		float pwr = 0;
			while(distance < driveTarget )
			{

					distance = abs(nMotorEncoder[LeftMWheel]);
			//Aaron Z trash code
					if( distance <= driveTarget / 2)
						pwr = distance / 5 + 20; //TODO: figure out how this works
					else
						pwr = ( driveTarget - distance ) / 10 + 20;

					//pwr = 64 * sin(distance * PI / driveTarget);

					pwr *= direction;
					setLeft(pwr);
					setRight(pwr);
			}
			setLeft(0);
			setRight(0);
		}

	// direction is binary -1 or 1, positive is right
	// time is milliseconds
	// degrees is always positive
	void spin(int time, int direction, int degrees) // we need to calculate degrees per tick
	{
		nMotorEncoder[LeftMWheel] = 0;
		// Must test

		while(nMotorEncoder[LeftBWheel] < direction * degrees)
		{
			motor[LeftBWheel] = motor[LeftMWheel] = motor[LeftFWheel] = direction * magnitude;
			motor[RightBWheel] = motor[RightMWheel] = motor[RightFWheel] = -direction * magnitude;
		}

		if(direction > 0)
			preciseDriveStop(RIGHT);
		else
			preciseDriveStop(LEFT);
	}

	void intake(bool direction)
	{
		if(direction == true)
			motor[LeftIntake] = motor[RightIntake] = 127;
		else if (direction == false)
			motor[LeftIntake] = motor[RightIntake] = -127;
	}

	void lift(int targetPot)
	{
		int currentPot = SensorValue[RightArmAngle]; // takes current Arm Angle
		while(currentPot != targetPot)
		{
			if(currentPot < targetPot)
			{
				motor[LeftArm] = motor[RightArm] = 127;	 //goes up if lower
			}
			/*
			else if(currentPot > targetPot)
			{
				motor[LeftArm] = motor[RightArm] = -127;	//goes down if higher
			}
			*/
			//wait1Msec(50);
			currentPot = SensorValue[RightArmAngle];
		}
	}

	void liftDown()
	{
		while(SensorValue[RightArmAngle] > LOW)
		{
			motor[LeftArm] = motor[RightArm] = -127;

		}
		motor[LeftArm] = motor[RightArm] = 0;
	}

	void holdArm()
	{
		motor[LeftArm] = motor[RightArm] = hold;
	}

	void stopIntake()
	{
		motor[LeftIntake] = motor[RightIntake] = 0;
	}

	void waitForButton()
	{
		while(SensorValue[waitingButton] == 0){}
	}

	void deploy()
	{
		intake(true);
		wait1Msec(300);
		stopIntake();
	}

	/* Psuedocode
			 Ramp Forward -> [Cross Barrier] -> Raise Arm HIGH -> Hold Arm HIGH -> Ramp Zero -> [Reach Goal] -> Outtake -> [Finish Outtake]
			 	-> Stop Intake -> Ramp Forward -> Ramp Zero -> [Reach Barrier] -> Lower Arm LOW -> Ramp Backward -> [Reach Square] -> Hard Zero
			 Wait Until Press -> Raise Arm BARRIER -> Hold Arm BARRIER -> Ramp Forward -> Ramp Zero -> [Reach Barrier] -> Ramp Backward
			 	-> [Reach Square] -> Hard Zero -> Wait Until Press -> Ramp Forward -> Ramp Zero -> [Reach Barrier]
	*/
	void Alex() // Caches preload (5) + Knocks 2 big balls (10) -- works sometimes
	{
		deploy();
		moveStraight(1, 0, 1550); // 1600 is just before goal...changed for some reason
		lift(HIGH); // nearest 100... may need to adjust HIGH value
		holdArm();
		moveStraight(1, 0, 300); // reaches goal
		//wait1Msec(1000);
		intake(false);
		wait1Msec(500); // outtake
		stopIntake();
		moveStraight(-1, 0, 400); // move back away from goal...Apparently Safety is greater than move forward
		liftDown();
		moveStraight(-1, 0, 1500);
		waitForButton();
		lift(BARRIER);
		holdArm();
		intake(false);
		moveStraight(1, 0, 550); // estimated guess based on 10Q's values - WORKS
		wait1Msec(300);
		moveStraight(-1, 0, 550);
		waitForButton();
		moveStraight(1, 0, 950); // A bit of trouble... Not sure if you want to spin rollers for this hit...
		wait1Msec(300); // outtaking pushes the ball away + needs good aiming
		moveStraight(-1, 0, 950);
		resetValues(100);
	}

	/* Pseudo
			Deploy -> Intake -> Ramp Forward -> Ramp Zero -> [Intake Two Balls] -> Hard Backward -> [Reach Square] -> Hard Zero
				Wait Until Press -> *** Outtake [Finish Outtake] -> Stop All -> Wait Until Press -> *** -> Intake -> Ramp Forward
				-> [Pick Up Ball] -> Ramp Zero -> Stop Intake -> [Reach Buckies] -> Spin -> [Knock Buckies] -> Outtake -> [Outtake Ball]
				-> Spin -> Intake -> Ramp Forward	-> [Pick Up Ball] -> Spin -> Outtake Ball
	*/
	void blueDevansh() // Places preload (1-2) + 2 buckies (2-4) + TURN RIGHT Knocks buckies (1-6) + Places two Balls (2)
	{
			deploy();
			intake(true);
			wait10Msec(10);
			moveStraight(1, 0, 150); // 200 estimated based on 10Q values
			wait10Msec(50);
			moveStraight(-1, 0, 150);
			stopIntake();
			waitForButton();
			lift(BUMP);
			holdArm();
			intake(false);
			waitForButton();
			resetValues(0);
			liftDown();
			wait10Msec(20);
			intake(true);
			moveStraight(1, 0, (TILE + HALF_TILE)); // theory
			wait10Msec(50); // intake ball hopefully
			lift(BUMP);
			holdArm();
			stopIntake();
			spin(0, 1, RIGHT_ANGLE); // BLUE SIDE TURN RIGHT knocks buckies hopefully
			wait10Msec(30);
			intake(false); // outtake ball hopefully
			wait10Msec(300);
			stopIntake();
			spin(0, -1, RIGHT_ANGLE);
			liftDown();
			intake(true);
			moveStraight(1, 0, (TILE + HALF_TILE)); // theory
			lift(BUMP);
			holdArm();
			stopIntake();
			spin(0, 1, RIGHT_ANGLE); // BLUE SIDE TURN RIGHT
			wait10Msec(30);
			intake(false);
			resetValues(10);
	}

	void redDevansh() // Places preload (1-2) + 2 buckies (2-4) + TURN LEFT Knocks buckies (1-6) + Places two Balls (2)
	{
			deploy();
			intake(true);
			wait10Msec(10);
			moveStraight(1, 0, 150); // 200 estimated based on 10Q values
			wait10Msec(50);
			moveStraight(-1, 0, 150);
			stopIntake();
			waitForButton();
			lift(BUMP);
			holdArm();
			intake(false);
			waitForButton();
			resetValues(0);
			liftDown();
			wait10Msec(20);
			intake(true);
			moveStraight(1, 0, (TILE + HALF_TILE)); // theory
			wait10Msec(50); // intake ball hopefully
			lift(BUMP);
			holdArm();
			stopIntake();
			spin(0, -1, RIGHT_ANGLE); // RED SIDE TURN LEFT knocks buckies hopefully
			wait10Msec(30);
			intake(false); // outtake ball hopefully
			wait10Msec(300);
			stopIntake();
			spin(0, 1, RIGHT_ANGLE);
			liftDown();
			intake(true);
			moveStraight(1, 0, (TILE + HALF_TILE)); // theory
			lift(BUMP);
			holdArm();
			stopIntake();
			spin(0, -1, RIGHT_ANGLE); // RED SIDE TURN LEFT
			wait10Msec(30);
			intake(false);
			resetValues(10);
	}

	/* Pseudo
		Deploy -> Intake -> Ramp Forward -> [Pick up Buckies] -> Ramp Backward -> [Go Over Bump] -> Wait
		-> Ramp Forward PRE_GOAL -> [Go under Barrier] -> Raise Arm HIGH -> Ramp Forward (GOAL-PRE_GOAL) -> Outtake
	*/

	void Udit()
	{
		deploy();
		intake(true);
		moveStraight(1, 0, 150);
		wait10Msec(30);
		moveStraight(-1, 0, 150);
		lift(BUMP);
		moveStraight(-1, 0, 150);
		setRight(127); setLeft(127);
	}

	void autonHangZoneAggro()
	{


	}

	void autonHangZoneDef()
	{

	}

	void autonMiddleZoneAggro()
	{


	}

	void autonMiddleZoneDef()
	{

	}

	void autonCustom()
	{

	}

	void autonProgrammingSkills()
	{

	}

	////////////////////////////////////////////// AUTON TESTING HERE //////////////////////////////////////////////////////////////

	void autonTest1()
	{
		Alex();
	}

	void autonTest2()
	{
		redDevansh();
	}

	void autonTest3()
	{
		blueDevansh();
	}

	void autonTest4()
	{
		Udit();
	}

	void autonTest5()
	{

	}

	void autonTest6()
	{

	}

	void autonTest6()
	{

	}

	void autonTest7()
	{

	}

	void autonTest8()
	{

	}



/*********************************************************************************************************************************
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Competition PROGRAM HERE ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
**********************************************************************************************************************************/
task autonomous()
{

}

task usercontrol() // I wonder if I can change this to userControl()... stupid vex names LOL
{
	while(true)
	{
		// Instance Variables
		int RightDrivePower, LeftDrivePower, LiftPower, IntakePower = 0;

		/////////////////////////////
		//*** DEAD ZONE CONTROL ***//
		/////////////////////////////
		int Channel3 = abs(vexRT(Ch3)) < control ? 0 : vexRT(Ch3); // deadzone setting - if abs(y) greater than 15 -> 0, else real value
		int Channel1 = abs(vexRT(Ch1)) < control ? 0 : vexRT(Ch1); // deadzone setting - if abs(x) greater than 15 -> 0, else real value

		///////////////////////////////
		//****** DRIVE CONTROL ******//
		///////////////////////////////

		// Halo Drive
			//Linear Scaling - Joseph has gotten used to this...

			    RightDrivePower = Channel3 - Channel1;
					LeftDrivePower = Channel3 + Channel1;

			//Soft Cubic Scaling - Joseph doesn't like the turning on this, but likes the extra precision... Sai might like
					/*
					RightDrivePower = cubicScaling(vexRT[Ch3]) - cubicScaling(vexRT[Ch1]);
					LeftDrivePower = cubicScaling(vexRT[Ch3]) + cubicScaling(vexRT[Ch1]);
					*/

		// Manual Tank - Devansh may like this
					/* //Soft Cubic Scaling
					RightDrivePower = cubicScaling(vexRT[Ch2]);
					LeftDrivePower = cubicScaling(vexRT[Ch3]);
					*/

					/* // Hard Cubic Scaling - Not Tested
					RightDrivePower = hardCubicScaling(vexRT[Ch2]);
					LeftDrivePower = hardCubicScaling(vexRT[Ch3]);
					*/

		///////////////////////////
		//***** LIFT CONTROL ****//
		///////////////////////////

		/**************************
		Liimits: Can't go lower than LOW
		Arm Holding: Constant hold between PRE_HIGH and HIGH
		**************************/

		// Logic order -> if no input and already raised -> Constant
		// 		If not -> Check Lowered Safety
		//		If not -> Manual Control
		//PRE-CONDITION: Arm Holding is Autonomous,
		//No user input, but user needs to take manual on demand

		if((vexRT[Btn5U] == 0 && vexRT[Btn5D] == 0) //High Arm hold
			&& (SensorValue[RightArmAngle] >= PRE_HIGH))
		{ // If no input and at the same time either arm is above PRE_HIGH...
			LiftPower = hold;
		}
		/*
		else if((vexRT[Btn5U] == 0 && vexRT[Btn5D] == 0)// Everywhere Arm Hold - Removed due to no use and wasted battery
			&& (SensorValue[RightArmAngle] >= BUMP|| SensorValue[LeftArmAngle] >= BUMP))
		{
			LiftPower = 25;// Arbitrary Numbers tested: 30 holds, 20 holds with noise, 25 holds fine
		}*/
		else if(SensorValue[RightArmAngle] <= LOW) //LOW Safety Limit
			LiftPower = vexRT[Btn5U]*127 - vexRT[Btn5D]*0; // can only go up now
		else // Full Manual
			LiftPower = vexRT[Btn5U]*127 - vexRT[Btn5D]*127;

		//////////////////////////
		//*** Intake Control ***//
		//////////////////////////
			IntakePower = vexRT[Btn6U]*127 - vexRT[Btn6D]*127;


		/*************************
		****** TESTING AREA ******
		*************************/

				if(vexRT[Btn8L]) // comment this before competition
				{
					autonTest1();
				}
				else if(vexRT[Btn8R])
				{
					autonTest2();
				}
				else if(vexRT[Btn8D])
				{
					autonTest3();
				}
				else if(vexRT[Btn8U])
				{
					autonTest4();
				}
				else if(vexRT[Btn7D])
				{
					autonTest5();
				}
				else if(vexRT[Btn7U])
				{
					autonTest6();
				}
				else if(vexRT[Btn7R])
				{
					autonTest7();
				}
				else if(vexRT[Btn7L])
				{
					autonTest8();
				}

				//Set motors to each individual powers...
				motor[RightBWheel] = RightDrivePower; // port 4
				motor[RightMWheel] = RightDrivePower;		// port 3
				motor[RightFWheel] = RightDrivePower;		// port 2
				//setRight(RightDrivePower);
				motor[LeftFWheel] = LeftDrivePower;	// port 9
				motor[LeftMWheel] = LeftDrivePower;	//port 8
				motor[LeftBWheel] = LeftDrivePower;	//port 7
				//setLeft(LeftDrivePower);

				motor[RightArm] = motor[LeftArm] = LiftPower;

				motor[RightIntake] = motor[LeftIntake] = IntakePower;

	} // end while update loop
} // end task usercontrol
