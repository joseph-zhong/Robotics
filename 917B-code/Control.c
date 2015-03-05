#define C1LX vexRT[Ch4]
#define C1LY -vexRT[Ch3]
#define C1RX vexRT[Ch1]
#define FORWARD 127
#define BACKWARD -127
#define STOP 0
//back right = 3
//back left = 2
//front left = 8
//front right = 9
long RightFrontDrive;
long RightBackDrive;
long LeftFrontDrive;
long LeftBackDrive;

int cubicMap(int x)
{
	return ((((x*3)/25)*((x*3)/25)*((x*3)/25)/27 + x/2)*2)/3;
}

//---------------------------------------------------------
task control()
{
	while(true)
	{
		motor[leftFrontMotor]=LeftFrontDrive;
		motor[rightFrontMotor]=RightFrontDrive;
		motor[rightBackMotor]=RightBackDrive;
		motor[leftBackMotor]=LeftBackDrive;

		int DriveFrontLeft=-1*(C1LY - C1LX - C1RX);
		int DriveFrontRight=-1*(-C1LY - C1LX - C1RX);
		int DriveBackRight=(-C1LY + C1LX - C1RX);
		int DriveBackLeft=(C1LY + C1LX - C1RX);
		
		float cubicPowerFrontLeft=cubicMap(DriveFrontLeft);
		float cubicPowerFrontRight=cubicMap(DriveFrontRight);
		float cubicPowerBackRight=cubicMap(DriveBackRight);
		float cubicPowerBackLeft=cubicMap(DriveBackLeft);
		
		RightFrontDrive = cubicPowerFrontRight;
		RightBackDrive = cubicPowerBackRight;
		LeftFrontDrive = cubicPowerFrontLeft;
		LeftBackDrive = cubicPowerBackLeft;
		
		abortTimeslice();
	}
}
