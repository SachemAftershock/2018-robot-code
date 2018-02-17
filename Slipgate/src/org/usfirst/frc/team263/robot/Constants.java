package org.usfirst.frc.team263.robot;

public class Constants {
	// CAN Bus Motor IDs
	final static int kLeftMasterDrivePort = 1;
	final static int kLeftSlaveDrivePort = 0;
	final static int kRightMasterDrivePort = 3;
	final static int kRightSlaveDrivePort = 2;

	// PCM Module ID
	final static int kDriveSolenoidPort = 0;

	// Drivebase constants
	final static double[] kTurningConstant = { 0.3, 0.5 };
	final static double[] kDriveKp = { 0.5, 0.00 };
	final static double[] kDriveKi = { 0.001, 0.00 };
	final static double[] kDriveKd = { 5.0, 0.00 };
	final static double[] kDriveKf = { 0.00, 0.00 };
	final static int[] kDriveAccel = { 1000, 1000 };
	final static int[] kDriveCruiseVelocity = { 5000, 5000 };
	final static int[] kDriveError = { 100, 10 };
	final static int[] kDriveIZone = { 200, 200 };
	final static int[] kDriveRampRate = { 256, 256 };
	final static double[] kCubeSeekSpeed = { 0.25, 0.45 };
	final static double kDriveWheelLength = 23.5;

	// Rotational PID Constants
	final static double[] kDriveRKp = { 0.005, 0.003 };
	final static double[] kDriveRKi = { 0.00, 0.000 };
	final static double[] kDriveRKd = { 0.04, 0.03 };
	final static double[] kDriveRKf = { 0.00, 0.00 };
	final static double[] kDriveRStaticFr = { 0.090, 0.095 };
	final static double[] kDriveREpsilon = { 0.5, 0.5 };

	// Curve PID Constants
	final static double[] kDriveCKp = { 0.005, 0.03 };
	final static double[] kDriveCKi = { 0.00, 0.000 };
	final static double[] kDriveCKd = { 0.00, 0.3 };
	final static double[] kDriveCKf = { 0.00, 0.00 };
	final static double[] kDriveCStaticFr = { 0.090, 0.095 };
	final static double[] kDriveCEpsilon = { 0.5, 0.5 };

	// Cube Intake constants
	final static int kLeftCubeWheel = 4;
	final static int kRightCubeWheel = 5;
	final static double kCubeWheelSpeed = 1;

	// Drivebase sensor/wheel constants
	final static double kUnitsPerRotationEnc = 4096.0 / 3.0;
	final static int kWheelRadiusInches = 3;
	final static double kWheelCircumference = 2 * kWheelRadiusInches * Math.PI;

	// DigitalInputs Port IDs
	final static int kCubeLeftLimitSwitch = 0;
	final static int kCubeRightLimitSwitch = 1;

	// Communication bus constants
	final static int kArduinoI2CAddress = 10;

	final static int kElevatorTopLimitSwitch = 2;
	final static int kElevatorBottomLimitSwitch = 3;

	// Elevator Constants
	final static double kElevatorKp = 0.0;
	final static double kElevatorKi = 0.0;
	final static double kElevatorKd = 0.0;
	final static double kElevatorKf = 0.0;
	final static int kMaxAccel = 0;
	final static int kMaxVel = 0;
	final static int kUnitsPerRotation = 4096;
	final static int kMinBufferSize = 10;
	final static int kItp = 10;
	final static int kT1 = 200;
	final static int kT2 = 100;
	final static int kVprog = 3000;
}
