package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import java.io.IOException;

import org.usfirst.frc.team263.robot.SWDrive.Direction;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot {
	XboxController pDriver, sDriver;
	SWDrive drive;
	CubeIntake intake;
	Logger logger;

	@Override
	public void robotInit() {
		pDriver = new XboxController(0);
		sDriver = new XboxController(1);
		intake = CubeIntake.getInstance();
		drive = SWDrive.getInstance();
		try {
			logger = new Logger();
		} catch (IOException e) {
			DriverStation.reportError("Couldn't instantiate logger", false);
		}
	}

	@Override
	public void teleopInit() {
		logger.write("Entering Teleoperated Mode", true);
		drive.setOpenLoop();
	}

	@Override
	public void teleopPeriodic() {
		drive.drive(pDriver);
		if (pDriver.getBumper(Hand.kLeft)) {
			drive.setCubeAssist(Direction.eCounterclockwise);
		} else if (pDriver.getBumper(Hand.kRight)) {
			drive.setCubeAssist(Direction.eClockwise);
		} else {
			drive.setOpenLoop();
		}
	}
	
	@Override
	public void disabledInit() {
		if (logger != null) {
			logger.forceSync();
		}
	}

	@Override
	public void autonomousInit() {
		logger.write("Entering Autonomous Mode", true);
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
