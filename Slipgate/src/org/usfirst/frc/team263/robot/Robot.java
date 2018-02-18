package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import java.io.IOException;

import org.usfirst.frc.team263.robot.Enums.AutoObjective;
import org.usfirst.frc.team263.robot.Enums.Direction;
import org.usfirst.frc.team263.robot.Enums.LEDMode;
import org.usfirst.frc.team263.robot.Limelight.CameraMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;

public class Robot extends TimedRobot {
	XboxController pDriver, sDriver;
	Elevator elevator;
	SWDrive drive;
	CubeIntake intake;
	Logger logger;
	Autonomous autonomous;
	Compressor compressor;
	
	@Override
	public void robotInit() {
		pDriver = new XboxController(0);
		sDriver = new XboxController(1);
		intake = CubeIntake.getInstance();
		elevator = Elevator.getInstance();
		drive = SWDrive.getInstance();
		autonomous = Autonomous.getInstance();
		compressor = new Compressor();

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
		LEDStrip.sendColor(LEDMode.eTeleop);
		compressor.setClosedLoopControl(true);
		compressor.start();
		if (pDriver.getBumper(Hand.kLeft)) {
			Limelight.setCameraMode(CameraMode.eVision);
			drive.setCubeAssist(Direction.eCounterclockwise);
		} else if (pDriver.getBumper(Hand.kRight)) {
			Limelight.setCameraMode(CameraMode.eVision);
			drive.setCubeAssist(Direction.eClockwise);
		} else {
			Limelight.setCameraMode(CameraMode.eDriver);
			drive.setOpenLoop();
		}

		if (pDriver.getAButton()) {
			drive.setHighGear();
			LEDStrip.sendColor(LEDMode.eHighGear);
		}
		if (pDriver.getXButton()) {
			LEDStrip.sendColor(LEDMode.eLowGear);
			drive.setLowGear();
		}
		
		if(RobotController.isBrownedOut()) {
			LEDStrip.sendColor(LEDMode.eBrownout);
		}

		drive.drive(pDriver);
		elevator.drive(sDriver);
	}

	@Override
	public void disabledInit() {
		if (logger != null) {
			logger.forceSync();
		}
		LEDStrip.sendColor(LEDMode.eRainbow);
	}

	@Override
	public void autonomousInit() {
		if (logger != null) {
			logger.write("Entering Autonomous Mode", true);
		}
		
		drive.zeroGyro();
		autonomous.clearQueue();
		LEDStrip.sendColor(LEDMode.eRainbow);
		
		autonomous.queueObjective(AutoObjective.eForward, 75);
		autonomous.queueObjective(AutoObjective.eRotate, 90);
		autonomous.queueObjective(AutoObjective.eForward, 120);
		autonomous.queueObjective(AutoObjective.eRotate, 180);
		autonomous.queueObjective(AutoObjective.eForward, 45);
		autonomous.queueObjective(AutoObjective.eRotate, 90);
		autonomous.queueObjective(AutoObjective.eForward, 80);
		autonomous.queueObjective(AutoObjective.eCubeAssist, 1);
	}

	@Override
	public void autonomousPeriodic() {
		autonomous.drive();
	}

	@Override
	public void testInit() {
		drive.setCurveControl(-20, 90);
		drive.zeroGyro();
	}

	@Override
	public void testPeriodic() {
		drive.drive(pDriver);
	}
}
