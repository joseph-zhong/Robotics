
task main()
	{
		deploy();
		intake(1);
		wait10Msec(210);
		moveStraight(1, 0, 455); //go over bump and picks up
		wait10Msec(250);
		moveStraight(-1, 0, 475);//comes back over bump
		intake(0);
		waitForButton();//reposition
		moveStraight(1, 0, 1600); // maintenence and recalibrating needed
		liftTime(1,300);
		holdArmHigh();
		moveStraight(1, 0, 650); // reaches goal
		//wait1Msec(1000);
		intake(-1);
		wait1Msec(500); // outtake
		resetValues(100);
	}
