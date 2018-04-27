package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import java.util.Arrays;

import org.usfirst.frc.team263.robot.Enums.AutoObjective;
import org.usfirst.frc.team263.robot.Enums.CIMode;
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
	boolean left, center, right, doSwitch;
	boolean hasTest;

	@Override
	public void robotInit() {
		System.loadLibrary("ProfileGeneratorJNI");
		leftJumper = new DigitalInput(5);
		centerJumper = new DigitalInput(6);
		rightJumper = new DigitalInput(7);

		left = false;// !leftJumper.get();
		center = true;// !centerJumper.get();
		right = false;// !rightJumper.get();
		doSwitch = true;

		hasTest = false;

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

		if (RobotController.isBrownedOut()) {
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
		if (c1 == 'L' && left && doSwitch) {
			autonomous.queueObjective(AutoObjective.eForward, 135);
			autonomous.queueObjective(AutoObjective.eRotate, 90);
			autonomous.queueObjective(AutoObjective.eForward, 10);
			autonomous.queueObjective(AutoObjective.eEjectCube, 0);
			autonomous.queueObjective(AutoObjective.eRotate, 0);
			autonomous.queueObjective(AutoObjective.eNothing, 0);
		} else if (c1 == 'R' && right && doSwitch) {
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
				autonomous.queueObjective(AutoObjective.eElevatorLevel, 3);
				autonomous.queueObjective(AutoObjective.eOpenArm, 0);
				autonomous.queueObjective(AutoObjective.eForward, 15);
				autonomous.queueObjective(AutoObjective.eIntake, 0);
				autonomous.queueObjective(AutoObjective.eForward, -15);
				autonomous.queueObjective(AutoObjective.eRotate, 0);
				autonomous.queueObjective(AutoObjective.eElevatorLevel, 4);
				autonomous.queueObjective(AutoObjective.eForward, 5);
				autonomous.queueObjective(AutoObjective.eEjectCube, 0);
				autonomous.queueObjective(AutoObjective.eNothing, 0);

			} else {
				autonomous.queueObjective(AutoObjective.eRotate, -60);
				autonomous.queueObjective(AutoObjective.eForward, 50);
				autonomous.queueObjective(AutoObjective.eRotate, 0);
				autonomous.queueObjective(AutoObjective.eForward, 45);
				autonomous.queueObjective(AutoObjective.eEjectCube, 0);
				autonomous.queueObjective(AutoObjective.eRotate, 90);
				autonomous.queueObjective(AutoObjective.eElevatorLevel, 3);
				autonomous.queueObjective(AutoObjective.eOpenArm, 0);
				autonomous.queueObjective(AutoObjective.eForward, 15);
				autonomous.queueObjective(AutoObjective.eIntake, 0);
				autonomous.queueObjective(AutoObjective.eForward, -15);
				autonomous.queueObjective(AutoObjective.eRotate, 0);
				autonomous.queueObjective(AutoObjective.eElevatorLevel, 4);
				autonomous.queueObjective(AutoObjective.eForward, 5);
				autonomous.queueObjective(AutoObjective.eEjectCube, 0);
				autonomous.queueObjective(AutoObjective.eNothing, 0);
			}
		} else if (left && c2 == 'L') {
			/*
			autonomous.queueObjective(AutoObjective.eForward, 240);
			autonomous.queueObjective(AutoObjective.eElevatorLevel, 6);
			autonomous.queueObjective(AutoObjective.eEjectCube, 0);
			autonomous.queueObjective(AutoObjective.eForward, -5);
			autonomous.queueObjective(AutoObjective.eElevatorLevel, 1);
			*/
		} else if (right && c2 == 'R') {
			/*
			autonomous.queueObjective(AutoObjective.eForward, 240);
			autonomous.queueObjective(AutoObjective.eElevatorLevel, 6);
			autonomous.queueObjective(AutoObjective.eEjectCube, 0);
			autonomous.queueObjective(AutoObjective.eForward, -5);
			autonomous.queueObjective(AutoObjective.eElevatorLevel, 1);
			*/
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
		drive.zeroGyro();
		autonomous.clearQueue();


	}

	@Override
	public void testPeriodic() {
		  ////////////////////
		 //// Drivebase /////
		////////////////////
		drive.zeroEncoders();
		
		for (int i = 250; i > 1; i--) {
			drive.drive(/*1.0 / i, 1.0 / i*/ 0.5, 0.5);
			Timer.delay(1.0 / 50.0);
		}
		
		drive.drive(0f,0f);
		
		Timer.delay(0.5);
		
		if(Math.abs(drive.getEncoders()[0]) < 500 || Math.abs(drive.getEncoders()[1]) < 500) {
			//System.out.println("Encoders: " + Arrays.toString(drive.getEncoders()));
			DriverStation.reportError("Encoders did not change!", false);
		}
		
		for (int i = 250; i > 1; i--) {
			//System.out.println("Encoders: " + Arrays.toString(drive.getEncoders()));
			drive.drive(/*-1.0 / i, -1.0 / i*/ -0.5, -0.5);
			Timer.delay(1.0 / 50.0);
		}
		
		drive.drive(0f,0f);
		
		Timer.delay(0.5);
		
		if (Math.abs(drive.getEncoders()[0] - drive.getEncoders()[1]) > 500) {
			//System.out.println("Encoders: " + Arrays.toString(drive.getEncoders()));
			//DriverStation.reportError("Left and right encoders do not match!", false);
			//TODO: fix...
		}
		
		if (Math.abs(drive.getEncoders()[0]) > 500 || Math.abs(drive.getEncoders()[1]) > 500) {
			//System.out.println("Encoders: " + Arrays.toString(drive.getEncoders()));
			DriverStation.reportError("Encoders did not zero", false);
		}
		
		  /////////////////////
		 ////// Intake ///////
		/////////////////////
		intake.drive(CIMode.eIn);
		Timer.delay(1);
		intake.drive(CIMode.eShoot);
		Timer.delay(1);
		intake.drive(CIMode.eStandby);
		System.out.println("Please verify the intake motors moved inwards then backwards.");
		while (!pDriver.getYButton()) {
			continue;
		}
		
		
		  /////////////////////
		 ///// Elevator //////
		/////////////////////
		elevator.zeroEncoder();
		elevator.test(0.5);
		if (Math.abs(elevator.getEncoder()) < 250) {
			DriverStation.reportError("Elevator encoder did not work!", false);
		}

		/////////////////////
		//// Pneumatics /////
		/////////////////////
		elevator.toggleClimberSolenoid();
		System.out.println("Please verify that elevator piston toggled");
		while (!pDriver.getYButton()) {
			continue;
		}
		
		elevator.toggleClimberSolenoid();
		System.out.println("Please verify that climber piston pushed out");
		while (!pDriver.getYButton()) {
			continue;
		}
		
		intake.drive(CIMode.eDrop);
		System.out.println("Please verify that intake toggled");
		while (!pDriver.getYButton()) {
			continue;
		}
		
		
		//elevator.toggleClimberSolenoid(); //TODO: toggleClimberSolenoid only goes one way
		
		//puts robot back to start posiiton
		elevator.toggleClimberSolenoid();
		intake.drive(CIMode.eDrop); //drop = toggle
		
		System.out.println("Health check complete!");
	}
	
	
}
