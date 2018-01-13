package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
	XboxController pDriver;
	SWDrive drive;

	@Override
	public void robotInit() {
		pDriver = new XboxController(0);
		drive = SWDrive.getInstance();
	}

	@Override
	public void teleopPeriodic() {
		drive.drive(pDriver);
	}
	
	@Override
	public void autonomousInit() {
		drive.driveSetDistance(12);
	}
	
	@Override
	public void autonomousPeriodic() {
		drive.driveDistance();
	}
}
