package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import java.util.Arrays;

import org.usfirst.frc.team263.robot.Enums.AutoObjective;
import org.usfirst.frc.team263.robot.Enums.Direction;
import org.usfirst.frc.team263.robot.Enums.LEDMode;
import org.usfirst.frc.team263.robot.Limelight.CameraMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotController;

public class Robot extends TimedRobot {
	XboxController pDriver, sDriver;
	MagicElevator elevator;
	SWDrive drive;
	CubeIntake intake;
	Logger logger;
	Autonomous autonomous;
	Compressor compressor;
	DigitalInput leftJumper, centerJumper, rightJumper;
	boolean left, center, right;
	
	@Override
	public void robotInit() {
		System.loadLibrary("ProfileGeneratorJNI");
		leftJumper = new DigitalInput(5);
		centerJumper = new DigitalInput(6);
		rightJumper = new DigitalInput(7);
		
		left = false;//!leftJumper.get();
		center = true;//!centerJumper.get();
		right = false;//!rightJumper.get();
		
		System.out.println("Left: " + left + " | Center: " + center + " | Right: " + right);
		
		pDriver = new XboxController(0);
		sDriver = new XboxController(1);
		intake = CubeIntake.getInstance();
		drive = SWDrive.getInstance();
		elevator = MagicElevator.getInstance();
		autonomous = Autonomous.getInstance();
		compressor = new Compressor();

		logger = new Logger();
	}
	
	@Override
	public void teleopInit() {
		logger.write("Entering Teleoperated Mode", true);
		drive.setOpenLoop();
	}

	@Override
	public void teleopPeriodic() {
		LEDStrip.sendColor(LEDMode.eRainbow);
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
			LEDStrip.sendColor(LEDMode.eBlink);
		}
		if (pDriver.getXButton()) {
			LEDStrip.sendColor(LEDMode.eBlink);
			drive.setLowGear();
		}
		
		if(RobotController.isBrownedOut()) {
			LEDStrip.sendColor(LEDMode.eBlink);
		}

		drive.drive(pDriver);
		elevator.drive(sDriver);
		intake.drive(sDriver);
	}

	@Override
	public void disabledInit() {
		logger.forceSync();
		LEDStrip.sendColor(LEDMode.eRainbow);
	}

	@Override
	public void autonomousInit() {
		Limelight.setCameraMode(CameraMode.eVision);
		logger.write("Entering Autonomous Mode", true);
		drive.zeroGyro();
		autonomous.clearQueue();
		LEDStrip.sendColor(LEDMode.eRainbow);
		
		char c1 = 'q';
		char c2 = 'q';
		while (c1 == 'q') {
			c1 = DriverStation.getInstance().getGameSpecificMessage().charAt(0);
			c2 = DriverStation.getInstance().getGameSpecificMessage().charAt(1);
		}
		autonomous.queueObjective(AutoObjective.eTriggerClimber, 0);
		/*if (c2 == 'L' && left) {
			autonomous.queueObjective(AutoObjective.eForward, 240);
			autonomous.queueObjective(AutoObjective.eRotate, 45);
			autonomous.queueObjective(AutoObjective.eElevatorLevel, 6);
			autonomous.queueObjective(AutoObjective.eEjectCube, 0);
			autonomous.queueObjective(AutoObjective.eNothing, 0);
		} else */if (c1 == 'L' && left) {
			System.out.println("THINGY");
			autonomous.queueObjective(AutoObjective.eForward, 135);
			autonomous.queueObjective(AutoObjective.eRotate, 90);
			autonomous.queueObjective(AutoObjective.eForward, 10);
			autonomous.queueObjective(AutoObjective.eEjectCube, 0);
			autonomous.queueObjective(AutoObjective.eRotate, 0);
			autonomous.queueObjective(AutoObjective.eNothing, 0);
		} else if (c1 == 'R' && right) {
			autonomous.queueObjective(AutoObjective.eForward, 135);
			autonomous.queueObjective(AutoObjective.eRotate, -90);
			autonomous.queueObjective(AutoObjective.eForward, 10);
			autonomous.queueObjective(AutoObjective.eEjectCube, 0);
			autonomous.queueObjective(AutoObjective.eRotate, 0);
			autonomous.queueObjective(AutoObjective.eNothing, 0);
		} else if (center) {
			autonomous.queueObjective(AutoObjective.eForward, 30);
			if (c1 == 'R') {
				autonomous.queueObjective(AutoObjective.eRotate, 60);
				autonomous.queueObjective(AutoObjective.eForward, 50);
				autonomous.queueObjective(AutoObjective.eRotate, 0);
				autonomous.queueObjective(AutoObjective.eForward, 45);
				autonomous.queueObjective(AutoObjective.eEjectCube, 0);
				autonomous.queueObjective(AutoObjective.eRotate, -90);
				//autonomous.queueObjective(AutoObjective.eElevatorLevel, 1);
				autonomous.queueObjective(AutoObjective.eOpenArm, 0);
				autonomous.queueObjective(AutoObjective.eForward, 15);
				autonomous.queueObjective(AutoObjective.eIntake, 0);
			} else {
				autonomous.queueObjective(AutoObjective.eRotate, -60);
				autonomous.queueObjective(AutoObjective.eForward, 50);
				autonomous.queueObjective(AutoObjective.eRotate, 0);
				autonomous.queueObjective(AutoObjective.eForward, 45);
				autonomous.queueObjective(AutoObjective.eEjectCube, 0);
				autonomous.queueObjective(AutoObjective.eRotate, 90);
				//autonomous.queueObjective(AutoObjective.eElevatorLevel, 1);
				autonomous.queueObjective(AutoObjective.eOpenArm, 0);
				autonomous.queueObjective(AutoObjective.eForward, 15);
				autonomous.queueObjective(AutoObjective.eIntake, 0);
			}
		} else {
			autonomous.queueObjective(AutoObjective.eForward, 135);
		}
	}

	@Override
	public void autonomousPeriodic() {
		autonomous.drive();
	}

	@Override
	public void testInit() {
		/*
		drive.zeroGyro();
		//drive.setLinearDistance(50);
		drive.setRotationTheta(90);
		try {
			double[] x = ProfileGeneratorJNI.createNewProfile(10, 500, 1000, 200, 40000);
			System.out.println(Arrays.toString(x));
		} catch (UnsatisfiedLinkError e) {
			System.err.println(e.getMessage());
		}
		*/
		drive.zeroGyro();
		autonomous.clearQueue();
		
	}

	@Override
	public void testPeriodic() {
		//drive.drive();
		//System.out.println("Yaw: "  + drive.getYaw() + ", PID: " + drive.getPid());
		//Timer.delay(0.5);
		//autonomous.drive();
		LEDStrip.sendColor(LEDMode.eBlue);
		if(pDriver.getAButton()) {
			System.out.println("HErro");
			LEDStrip.sendColor(LEDMode.eExpel);
		}
	}
}
