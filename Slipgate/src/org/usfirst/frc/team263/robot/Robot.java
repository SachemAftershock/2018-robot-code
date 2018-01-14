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
	public void teleopInit() {
		drive.setOpenLoop();
	}
	
	@Override
	public void teleopPeriodic() {
		drive.drive(pDriver);
	}
	
	@Override
	public void autonomousInit() {
		drive.setLinearDistance(12);
	}
	
	@Override
	public void autonomousPeriodic() {
		drive.drive(pDriver);
	}
	
	@Override
	public void testInit() {
		drive.zeroGyro();
		drive.setRotationTheta(-35);
	}
	
	@Override
	public void testPeriodic() {
		drive.drive(pDriver);
	}
}
