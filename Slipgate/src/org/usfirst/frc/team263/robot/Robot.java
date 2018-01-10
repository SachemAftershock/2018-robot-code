package org.usfirst.frc.team263.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
	XboxController pDriver;
	TalonSRX leftBack, leftFront, rightBack, rightFront;
	SWDrive drive;

	@Override
	public void robotInit() {
		pDriver = new XboxController(0);
		
		leftBack = new TalonSRX(0);
		leftFront = new TalonSRX(1);
		rightBack = new TalonSRX(2);
		rightFront = new TalonSRX(3);
		
		drive = new SWDrive(leftBack, leftFront, rightBack, rightFront);
	}

	@Override
	public void teleopPeriodic() {
		drive.drive(pDriver);
	}
}
