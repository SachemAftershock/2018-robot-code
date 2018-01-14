package org.usfirst.frc.team263.robot;

public class Constants {
	final static int kLeftMasterDrivePort = 3;
	final static int kLeftSlaveDrivePort = 0;
	final static int kRightMasterDrivePort = 2;
	final static int kRightSlaveDrivePort = 1;
	final static double kTurningConstant = 0.5;
	final static double kDriveKp = 0.125;
	final static double kDriveKi = 0.0;
	final static double kDriveKd = 0.05;
	final static double kDriveKf = 0.0;
	final static double kDriveRKp = 0.005;
	final static double kDriveRKi = 0.00001;
	final static double kDriveRKd = 0.001;
	final static double kDriveRKf = 0.00;
	final static double kDriveREpsilon = 5;
	final static int kDriveAccel = 1000;
	final static int kDriveCruiseVelocity = 5000;
	final static int kDriveError = 100;
	final static int kDriveIZone = 200;
	final static int kDriveRampRate = 256;
	final static int kUnitsPerRotationEnc = 4096;
	final static int kWheelRadiusInches = 3;
	final static double kWheelCircumference = 2 * kWheelRadiusInches * Math.PI;
}
