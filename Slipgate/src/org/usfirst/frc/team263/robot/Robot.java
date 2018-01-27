package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import java.io.IOException;

import org.usfirst.frc.team263.robot.Enums.AutoObjective;
import org.usfirst.frc.team263.robot.Enums.Direction;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot {
	XboxController pDriver, sDriver;
	SWDrive drive;
	CubeIntake intake;
	Logger logger;
	Autonomous autonomous;

	@Override
	public void robotInit() {
		pDriver = new XboxController(0);
		sDriver = new XboxController(1);
		intake = CubeIntake.getInstance();
		drive = SWDrive.getInstance();
		autonomous = Autonomous.getInstance();
		
		try {
			logger = new Logger();
		} catch (IOException e) {
			DriverStation.reportError("Couldn't instantiate logger", false);
		}
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
		autonomous.queueObjective(AutoObjective.eForward, 12);
	}

	@Override
	public void autonomousPeriodic() {
		autonomous.drive();
	}

	@Override
	public void testInit() {
		drive.setOpenLoop();
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
