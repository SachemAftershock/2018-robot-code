package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot {
	XboxController pDriver, sDriver;
	SWDrive drive;
	CubeIntake intake;

	@Override
	public void robotInit() {
		pDriver = new XboxController(0);
		sDriver = new XboxController(1);
		intake = CubeIntake.getInstance();
		drive = SWDrive.getInstance();
	}

	@Override
	public void teleopInit() {
		drive.setOpenLoop();
	}
	
	@Override
	public void teleopPeriodic() {
		drive.drive(pDriver);
		if (pDriver.getBumper(Hand.kLeft) && pDriver.getBumper(Hand.kRight)) {
			drive.setCubeAssist();
		} else {
			drive.setOpenLoop();
		}
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
		drive.setRotationTheta(-90);
	}
	
	@Override
	public void testPeriodic() {
		drive.drive(pDriver);
		intake.drive(sDriver);
	}
}
