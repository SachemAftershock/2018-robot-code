package org.usfirst.frc.team263.robot;

public class Constants {
	final static int kLeftMasterDrivePort = 3;
	final static int kLeftSlaveDrivePort = 2;
	final static int kRightMasterDrivePort = 4;
	final static int kRightSlaveDrivePort = 1;
	final static double kTurningConstant = 0.5;
	final static double kDriveKp = 0.00;
	final static double kDriveKi = 0.00;
	final static double kDriveKd = 0.00;
	final static double kDriveKf = 0.0;
	final static double kDriveRKp = 0.006;
	final static double kDriveRKi = 0.000;
	final static double kDriveRKd = 0.000;
	final static double kDriveRKf = 0.00;
	final static double kDriveRStaticFr = 0.1;
	final static double kDriveREpsilon = 2;
	final static double kCubeSeekSpeed = 0.3;
	final static int kDriveAccel = 1000;
	final static int kDriveCruiseVelocity = 5000;
	final static int kDriveError = 100;
	final static int kDriveIZone = 200;
	final static int kDriveRampRate = 256;
	final static int kUnitsPerRotationEnc = 4096;
	final static int kWheelRadiusInches = 3;
	final static double kWheelCircumference = 2 * kWheelRadiusInches * Math.PI;
	final static int kLeftCubeWheel = 4;
	final static int kRightCubeWheel = 5;
	final static double kCubeWheelSpeed= 0.2;
}
