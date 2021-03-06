#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    Gyro,           sensorGyro)
#pragma config(Sensor, in3,    PotentL,        sensorPotentiometer)
#pragma config(Sensor, in4,    PotentR,        sensorPotentiometer)
#pragma config(Sensor, dgtl3,  Switch0,        sensorTouch)
#pragma config(Sensor, dgtl4,  Switch1,        sensorTouch)
#pragma config(Sensor, dgtl5,  Switch2,        sensorTouch)
#pragma config(Sensor, dgtl6,  TowerLimitL,    sensorTouch)
#pragma config(Sensor, dgtl7,  TowerLimitR,    sensorTouch)
#pragma config(Sensor, dgtl8,  ButtonBlockR,   sensorTouch)
#pragma config(Sensor, dgtl9,  ButtonWait,     sensorTouch)
#pragma config(Sensor, dgtl10, ButtonBlockL,   sensorTouch)
#pragma config(Sensor, dgtl11, TrayPiston,     sensorDigitalOut)
#pragma config(Sensor, dgtl12, PlatformPiston, sensorDigitalOut)
#pragma config(Sensor, I2C_1,  EncoderR,       sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           DriveRB,       tmotorVex393HighSpeed, openLoop, reversed)
#pragma config(Motor,  port2,           ArmRT,         tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port3,           RollerR,       tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port4,           ArmRB,         tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port5,           DriveRF,       tmotorVex393HighSpeed, openLoop, reversed)
#pragma config(Motor,  port6,           DriveLF,       tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port7,           ArmLB,         tmotorVex393, openLoop)
#pragma config(Motor,  port8,           RollerL,       tmotorVex393HighSpeed, openLoop, reversed)
#pragma config(Motor,  port9,           ArmLT,         tmotorVex393, openLoop)
#pragma config(Motor,  port10,          DriveLB,       tmotorVex393HighSpeed, openLoop, encoder, encoderPort, I2C_1, 1000)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//-----------------------------------------------------------
//			AUTON
//-------------------------------------.----------------------

void foldOut()
{
	SensorValue[PlatformPiston] = 1;
}

void autonWait()
{
	while( SensorValue[ButtonBlockR] != 1 &&
		SensorValue[ButtonBlockL] != 1 &&
		SensorValue[TowerLimitL] != 1 &&
		SensorValue[TowerLimitR] != 1)
	{}
}

void movePiston( int state_1, int state_2 )
{
	SensorValue[TrayPiston] = state_1;
	SensorValue[PlatformPiston] = state_2;
}

void moveRollers( int power_L, int power_R )
{
	motor[RollerL] = power_L;
	motor[RollerR] = power_R;
}

void moveArm( int power_L, int power_R )
{
	motor[ArmLB] = motor[ArmLT] = power_L;
	motor[ArmRB] = motor[ArmRT] = power_R;
}

void moveDrive( int power_L, int power_R )
{
	motor[DriveLB] = motor[DriveLF] = power_L;
	motor[DriveRB] = motor[DriveRF] = power_R;
}

void stopMotors()
{
	motor[ArmLB] = motor[ArmLT] = motor[ArmRB] = motor[ArmRT] = 0;
	motor[DriveLB] = motor[DriveLF] = motor[DriveRB] = motor[DriveRF] = 0;
	motor[RollerL] = motor[RollerR] = 0;
}

void resetEncoders()
{
	//SensorValue[EncoderL] = 0;
	SensorValue[EncoderR] = 0;
}

void resetGyro()
{
	SensorType[Gyro] = sensorNone;
  wait10Msec(50);

  SensorType[Gyro] = sensorGyro;
  wait10Msec(50);
}

int old_gyro_value = 0;

void artificiallyresetGyro()
{
	old_gyro_value = SensorValue[Gyro];
}



//int potentLValue;
int getPotentLValue()
{
	return SensorValue[PotentL] - 1170;
}





//	TOWER PRIMITIVES

float integral_A;
int prev_A = 0;

void setArm( int point )
{
	float maxPower = 127.0;																																	//	Limit Constants
	float integralLimit = 20.0;

	float P = .31415;																																			//	PID Constants
	float I = 0.0003;
	float D = 0.25;

	float error = point - getPotentLValue();																						//	PID Calculations
	integral_A += error;
	float derivative = error - prev_A;

	if( abs( error ) > integralLimit || error == 0 )														//	Integral Limit Filter
		integral_A = 0;

	prev_A = error;																																	//	Derivative Reset

	float power = P * error + I * integral_A + D * derivative;						//	Power Calculation

	if( abs( power ) > maxPower )																									//	Power Limit Filter
		power = power < 0 ? .9 * -maxPower : maxPower;

	moveArm( power, power );
}

int getGyroValue()
{
	return -1 * SensorValue[Gyro];
}

float integral_D;
int prev_D = 0;

void spinDrive( int point )
{
	float maxPower = 100.0;																																	//	Limit Constants
	float integralLimit = 80.0;

	float P = 0.42;																																			//	PID Constants
	float I = 0.001;
	float D = 0.0;

	float error = point - SensorValue[EncoderR];																						//	PID Calculations
	integral_D += error;
	float derivative = error - prev_D;

	if( abs( error ) > integralLimit || error == 0 )														//	Integral Limit Filter
		integral_D = 0;

	prev_D = error;																																	//	Derivative Reset

	float power = P * error + I * integral_D + D * derivative;						//	Power Calculation

	if( abs( power ) > maxPower )																									//	Power Limit Filter
		power = power < 0 ? -maxPower : maxPower;

	moveDrive( power, -power );
}

void wheelDrive( int point, char direction )
{
	float maxPower = 60.0;																																	//	Limit Constants
	float integralLimit = 80.0;

	float P = 0.42;																																			//	PID Constants
	float I = 0.001;
	float D = 0.0;

	float error = point - SensorValue[EncoderR];																						//	PID Calculations
	integral_D += error;
	float derivative = error - prev_D;

	if( abs( error ) > integralLimit || error == 0 )														//	Integral Limit Filter
		integral_D = 0;

	prev_D = error;																																	//	Derivative Reset

	float power = P * error + I * integral_D + D * derivative;						//	Power Calculation

	if( abs( power ) > maxPower )																									//	Power Limit Filter
		power = power < 0 ? -maxPower : maxPower;

	if(direction == 'R')
		moveDrive( power, 0 );
	else
		moveDrive( 0, power);
}

void setDrive( int point )
{
	float maxPower = 120.0;																																	//	Limit Constants
	float integralLimit = 50.0;

	float P = 0.118;																																			//	PID Constants
	float I = 0.00075;
	float D = 0.0;

	float error = point - SensorValue[EncoderR];																						//	PID Calculations
	integral_D += error;
	float derivative = error - prev_D;

	if( abs( error ) > integralLimit || error == 0 )														//	Integral Limit Filter
		integral_D = 0;

	prev_D = error;																																	//	Derivative Reset

	float power = P * error + I * integral_D + D * derivative;						//	Power Calculation

	if( abs( power ) > maxPower )																									//	Power Limit Filter
		power = power < 0 ? -maxPower : maxPower;

	moveDrive( power, power );
}





//	DRIVE PRIMITIVES






int filteredGyro = SensorValue[Gyro];
int previousGyro = filteredGyro;
int currentGyro = filteredGyro;
int drift = 0;

/*void gyroCorrect ( )
{
	currentGyro = SensorValue[Gyro];

	if (abs(currentGyro - previousGyro) < drift_d)
	{
		drift += (currentGyro - previousGyro);
	}

	else
	{
		filteredGyro = currentGyro - drift;
	}

	previousGyro = currentGyro;
}*/




void setRobot( int distance, char direction, int height, int roller, int tray, int platform, int timer )
{
	ClearTimer( T1 );
	resetEncoders();
	//artificiallyresetGyro();

	while( time1[T1] < timer )
	{
		if( direction == 'S' )
			setDrive( distance );
		else if( direction == 'T' )
			spinDrive( distance );
		else
			wheelDrive( distance, direction );
		//setArm( height );
		if( height == 0 )
			moveArm( -10, -10 );
		else
		{
			if( SensorValue[ButtonBlockL] == 1 && SensorValue[ButtonBlockR] == 1 )
				moveArm( 7, 7 );
			else if( SensorValue[ButtonBlockL] == 1 && SensorValue[ButtonBlockR] == 0 )
				moveArm( 7, 20 );
			else if( SensorValue[ButtonBlockL] == 0 && SensorValue[ButtonBlockR] == 1 )
				moveArm( 20, 7 );
			else
				setArm( height );
		}
		moveRollers( roller, roller );
		movePiston( tray, platform );
	}

	stopMotors();
}

void setRobotPhysLimit( int distance, char direction, int height, int roller, int tray, int platform, int timer )
{
	ClearTimer( T1 );
	resetEncoders();
	//artificiallyresetGyro();

	while(time1[T1] < timer && !SensorValue[TowerLimitL] && !SensorValue[TowerLimitR]  )
	{
		if( direction == 'S' )
			setDrive( distance );
		else if( direction == 'T' )
			spinDrive( distance );
		else
			wheelDrive( distance, direction );
		//setArm( height );
		if( height == 0 )
			moveArm( -10, -10 );
		else
		{
			if( SensorValue[ButtonBlockL] == 1 && SensorValue[ButtonBlockR] == 1 )
				moveArm( 7, 7 );
			else if( SensorValue[ButtonBlockL] == 1 && SensorValue[ButtonBlockR] == 0 )
				moveArm( 7, 20 );
			else if( SensorValue[ButtonBlockL] == 0 && SensorValue[ButtonBlockR] == 1 )
				moveArm( 20, 7 );
			else
				setArm( height );
		}
		moveRollers( roller, roller );
		movePiston( tray, platform );
	}

	stopMotors();
}

//-----------------------------------------------------------
//			DRIVE
//-----------------------------------------------------------

void manualDrive()
{
	if( abs( vexRT(Ch3) ) > 15 )
		motor[DriveLF] = motor[DriveLB] = vexRT(Ch3);
	else
		motor[DriveLF] = motor[DriveLB] = 0;

	if( abs( vexRT(Ch2) ) > 15 )
		motor[DriveRB] = motor[DriveRF] = vexRT(Ch2);
	else
		motor[DriveRB] = motor[DriveRF] = 0;
}

void manualArmUpSimple()
{
	if( SensorValue[ButtonBlockL] == 0 && SensorValue[ButtonBlockR] == 0 )
		moveArm( 127, 127 );
	else if( SensorValue[ButtonBlockL] == 1 && SensorValue[ButtonBlockR] == 0 )
		moveArm( 3, 20 );
	else if( SensorValue[ButtonBlockL] == 0 && SensorValue[ButtonBlockR] == 1 )
		moveArm( 20, 3 );
	else
		moveArm( 3, 3 );
}

void manualArmDownSimple()
{
	moveArm( -40, -40 );
}

void manualArmUpComplex()
{
	if( SensorValue[PotentL] < SensorValue[PotentR] )
		moveArm( (int)( 127.0 ), (int)( 127.0 * 0.9 ) );
	else if( SensorValue[PotentL] > SensorValue[PotentR] )
		moveArm( (int)( 127.0 * 0.9 ), (int)( 127.0 ) );
	else
		moveArm( (int)( 127.0 ), (int)( 127.0 ) );
}

void manualArmDownComplex()
{
	if( SensorValue[PotentL] > SensorValue[PotentR] )
		moveArm( (int)( -40.0 ), (int)( -40.0 * 0.9 ) );
	else if( SensorValue[PotentL] < SensorValue[PotentR] )
		moveArm( (int)( -40.0 * 0.9 ), (int)( -40.0 ) );
	else
		moveArm( (int)( -40.0 ), (int)( -40.0 ) );
}

bool toggleState_Tray;
bool toggleState_Platform;
int toggleDelay = 250;

void manualFeed()
{
	if( getPotentLValue() > 1200 )
	{
		if( vexRT[Btn6D] == 1 )
			moveRollers( 127, 127 );
		else if( vexRT[Btn6U] == 1 )
		{
			moveRollers( -127, -127 );
			toggleState_Tray = true;
		}
		else
			moveRollers( 0, 0 );

		if( vexRT[Btn8D] == 1 )
		{
			wait1Msec( toggleDelay );
			if( vexRT[Btn8D] == 0 )
				toggleState_Tray = toggleState_Tray ? false : true;
		}

	}
	else if( getPotentLValue() > 500 && getPotentLValue() <= 1200 )
	{
		toggleState_Tray = false;

		if( vexRT[Btn6D] == 1 )
			moveRollers( 127, 127 );
		else if( vexRT[Btn6U] == 1 && vexRT[Btn8D] == 1 )
		{
			toggleState_Tray = true;
			moveRollers( -127, -127 );
		}
		else if( vexRT[Btn6U] == 1 )
			moveRollers( -127, -127 );
		else if( vexRT[Btn8D] == 1 )
			toggleState_Tray = true;
		else
			moveRollers( 0, 0 );

	}
	else
	{
		toggleState_Tray = false;

		if( vexRT[Btn6D] == 1 )
			moveRollers( 127, 127 );
		else if( vexRT[Btn6U] == 1 && vexRT[Btn8D] == 1 )
		{
			toggleState_Tray = true;
			moveRollers( -127, -127 );
		}
		else if( vexRT[Btn6U] == 1 )
			moveRollers( -127, -127 );
		else if( vexRT[Btn8D] == 1 )
			toggleState_Tray = true;
		else
			moveRollers( 0, 0 );
	}
}

void pneumaticsCode()
{
	if( vexRT[Btn8U] == 1 )
		{
			wait1Msec( toggleDelay );
			if( vexRT[Btn8U] == 0 )
				toggleState_Platform = toggleState_Platform ? false : true;
		}
	SensorValue[TrayPiston] = toggleState_Tray ? 1 : 0;
	SensorValue[PlatformPiston] = toggleState_Platform ? 1 : 0;
}

//-----------------------------------------------------------
