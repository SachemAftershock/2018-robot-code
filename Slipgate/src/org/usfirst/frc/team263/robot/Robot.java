package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import java.io.IOException;
import java.util.Arrays;

import org.usfirst.frc.team263.robot.Enums.Direction;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot {
	XboxController pDriver, sDriver;
	SWDrive drive;
	Elevator elevator;
	CubeIntake intake;
	Logger logger;

	@Override
	public void robotInit() {
		pDriver = new XboxController(0);
		sDriver = new XboxController(1);
		intake = CubeIntake.getInstance();
		drive = SWDrive.getInstance();
		elevator = Elevator.getInstance();
		try {
			logger = new Logger();
		} catch (IOException e) {
			DriverStation.reportError("Couldn't instantiate logger", false);
		}
		
		System.loadLibrary("ProfileGeneratorJNI");
	}

	@Override
	public void teleopInit() {
		if (logger != null) {
			logger.write("Entering Teleoperated Mode", true);
		}
		drive.setOpenLoop();
	}

	@Override
	public void teleopPeriodic() {
		drive.drive(pDriver);
		elevator.drive(sDriver);
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
		if (logger != null) {
			logger.write("Entering Autonomous Mode", true);
		}
		drive.setLinearDistance(12);
	}

	@Override
	public void autonomousPeriodic() {
		drive.drive(pDriver);
	}

	@Override
	public void testInit() {
		drive.setOpenLoop();
		try {
			double[] x = ProfileGeneratorJNI.createNewProfile(10, 500, 1000, 200, 40000);
			System.out.println(Arrays.toString(x));
		} catch (UnsatisfiedLinkError e) {
			System.err.println(e.getMessage());
		}
	}

	@Override
	public void testPeriodic() {
		if (pDriver.getAButton()) {
			drive.setHighGear();
		}
		if (pDriver.getXButton()) {
			drive.setLowGear();
		}
	}
}
