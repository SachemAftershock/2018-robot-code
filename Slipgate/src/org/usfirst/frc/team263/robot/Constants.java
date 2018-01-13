package org.usfirst.frc.team263.robot;

public class Constants {
	final static int kLeftMasterDrivePort = 0;
	final static int kLeftSlaveDrivePort = 1;
	final static int kRightMasterDrivePort = 2;
	final static int kRightSlaveDrivePort = 3;
	final static double kTurningConstant = 0.5;
	final static double kDriveKp = 1.0;
	final static double kDriveKi = 0.0;
	final static double kDriveKd = 0.0;
	final static double kDriveKf = 0.0;
	final static int kDriveIZone = 200;
	final static int kDriveRampRate = 256;
	final static int kUnitsPerRotationEnc = 4096;
	final static int kWheelRadiusInches = 3;
	final static double kWheelCircumference = 2 * kWheelRadiusInches * Math.PI;
}
