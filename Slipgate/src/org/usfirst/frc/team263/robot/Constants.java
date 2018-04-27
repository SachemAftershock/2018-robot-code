package org.usfirst.frc.team263.robot;

public class Constants {
	// CAN Bus Motor IDs
	final static int kLeftMasterDrivePort = 3;
	final static int kLeftSlaveDrivePort = 1;
	final static int kRightMasterDrivePort = 4;
	final static int kRightSlaveDrivePort = 2;
	final static int kRampVictorPort = 0;

	// PCM Module ID
	final static int kDriveSolenoidPort = 4;

	// Drivebase constants
	final static double[] kTurningConstant = { 0.5, 0.5 };
	final static double[] kDriveKp = { 0.25, 0.00 };
	final static double[] kDriveKi = { 0.00, 0.00 };
	final static double[] kDriveKd = { 3.0, 0.00 };
	final static double[] kDriveKf = { 0.00, 0.00 };
	final static int[] kDriveAccel = { 1000, 1000 };
	final static int[] kDriveCruiseVelocity = { 5000, 5000 };
	final static int[] kDriveError = { 200, 10 }; //[0] was 100, 
	final static int[] kDriveIZone = { 200, 200 };
	final static int[] kDriveRampRate = { 256, 256 };
	final static double[] kCubeSeekSpeed = { 0.60, 0.45 };
	final static double kDriveWheelLength = 23.5;
	final static double kDriveMultiplier = 0.25;
	final static double kAccelPercentPerLoop = 0.04;
	final static double kAccelLimHeight = 0.5;

	// Rotational PID Constants
	//final static double[] kDriveRKp = { 0.010, 0.003 };
	final static double[] kDriveRKp = { 0.0105, 0.003 };
	final static double[] kDriveRKi = { 0.000, 0.000 };
	final static double[] kDriveRKd = { 0.030, 0.03 };
	final static double[] kDriveRKf = { 0.00, 0.00 };
	final static double[] kDriveRStaticFr = { 0.175, 0.095 };
	//final static double[] kDriveREpsilon = { 1.0, 0.5 };
	final static double[] kDriveREpsilon = { 1.5, 0.5 };

	// Curve PID Constants
	final static double[] kDriveCKp = { 0.005, 0.03 };
	final static double[] kDriveCKi = { 0.00, 0.000 };
	final static double[] kDriveCKd = { 0.00, 0.3 };
	final static double[] kDriveCKf = { 0.00, 0.00 };
	final static double[] kDriveCStaticFr = { 0.090, 0.095 };
	final static double[] kDriveCEpsilon = { 0.5, 0.5 };

	// Cube Intake constants
	final static int kLeftCubeWheel = 0;
	final static int kRightCubeWheel = 1;
	final static double kCubeWheelSpeed = 1;
	final static int kCubeSolFwd = 3; //5
	final static int kCubeSolRev = 4; //2
	final static double kCubeCurrentThresh = 5;

	// Drivebase sensor/wheel constants
	final static double kUnitsPerRotationEnc = 4096.0 / 3.0;
	final static int kWheelRadiusInches = 3;
	final static double kWheelCircumference = 2 * kWheelRadiusInches * Math.PI;
	final static int kLRampSolFwd = 3;
	final static int kLRampSolRev = 4;
	final static int kRRampSolFwd = 0;
	final static int kRRampSolRev = 7;

	// Communication bus constants
	final static int kArduinoI2CAddress = 10;

	// Elevator Constants
	final static double kElevatorKp = 0.01;
	final static double kElevatorKi = 0.0;
	final static double kElevatorKd = 0.0;
	final static double kElevatorKf = 0.0;
	final static int kMaxAccel = 0;
	final static int kMaxVel = 0;
	final static int kElevatorUnitsPerRotation = 5120;
	final static double kDistancePerRotationInches = 5.5;
	final static double kElevatorRotationsPerInch = kElevatorUnitsPerRotation / kDistancePerRotationInches;
	final static int kElevatorThreshhold = 10;
	final static int kInitialCount = 2420;
	final static int kMinBufferSize = 10;
	final static double kItp = 10;
	final static double kT1 = 200;
	final static double kT2 = 100;
	final static double kVprog = 10;
	final static int kOverride = 4;
	final static int kElevatorTalon = 2;
	final static double kMinimumVelocityInPercentThatOvercomesMotorInertia = 0.10;
	final static int kElevatorSolFwd = 6;
	final static int kElevatorSolRev = 1;
	final static int kClimberSolFwd = 2;
	final static int kClimberSolRev = 5;
	final static int kClimberVictor = 0;
	
	final static int kTiltThresh = 15;
}
