package org.usfirst.frc.team263.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI;

/**
 * Drivebase code for a six-wheel WC-style drive.
 * 
 * @version 0.0.2
 * @author Dan Waxman
 * @since 01-06-2018
 */
public class SWDrive {
	private static SWDrive mInstance = new SWDrive();
	private TalonSRX mLeftMaster, mLeftSlave, mRightMaster, mRightSlave;
	private AHRS mNavX;
	private double leftSetpoint, rightSetpoint;

	public static SWDrive getInstance() {
		return mInstance;
	}

	/**
	 * Constructor for SWDrive class.
	 */
	private SWDrive() {
		leftSetpoint = 0;
		rightSetpoint = 0;
		
		mLeftMaster = new TalonSRX(Constants.kLeftMasterDrivePort);
  		mLeftMaster.setNeutralMode(NeutralMode.Brake);
		mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		mLeftMaster.setSelectedSensorPosition(0, 0, 0);
		mLeftMaster.setSensorPhase(true);
		mLeftMaster.setInverted(false);
		mLeftMaster.config_kP(0, Constants.kDriveKp, 0);
		mLeftMaster.config_kI(0, Constants.kDriveKi, 0);
		mLeftMaster.config_kD(0, Constants.kDriveKd, 0);
		mLeftMaster.config_kF(0, Constants.kDriveKf, 0);
		mLeftMaster.config_IntegralZone(0, Constants.kDriveIZone, 0);
		mLeftMaster.configClosedloopRamp(0, Constants.kDriveRampRate);
		mLeftMaster.configOpenloopRamp(0, Constants.kDriveRampRate);
		mLeftMaster.configAllowableClosedloopError(0, Constants.kDriveError, 0);

		mLeftSlave = new TalonSRX(Constants.kLeftSlaveDrivePort);
		mLeftSlave.setInverted(true);
		mLeftSlave.setNeutralMode(NeutralMode.Brake);
		mLeftSlave.follow(mLeftMaster);

		mRightMaster = new TalonSRX(Constants.kRightMasterDrivePort);
		mRightMaster.setNeutralMode(NeutralMode.Brake);
		mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		mRightMaster.setSelectedSensorPosition(0, 0, 0);
		mRightMaster.setSensorPhase(true);
		mRightMaster.setInverted(true);
		mRightMaster.config_kP(0, Constants.kDriveKp, 0);
		mRightMaster.config_kI(0, Constants.kDriveKi, 0);
		mRightMaster.config_kD(0, Constants.kDriveKd, 0);
		mRightMaster.config_kF(0, Constants.kDriveKf, 0);
		mRightMaster.config_IntegralZone(0, Constants.kDriveIZone, 0);
		mRightMaster.configClosedloopRamp(0, Constants.kDriveRampRate);
		mRightMaster.configOpenloopRamp(0, Constants.kDriveRampRate);
		mRightMaster.configAllowableClosedloopError(0, Constants.kDriveError, 0);

		mRightSlave = new TalonSRX(Constants.kRightSlaveDrivePort);
		mRightSlave.setInverted(true);
		mRightSlave.setNeutralMode(NeutralMode.Brake);
		mRightSlave.follow(mRightMaster);

		mNavX = new AHRS(SPI.Port.kMXP);
	}

	/**
	 * Method to drive robot given controller of primary driver.
	 * 
	 * @param controller
	 *            Primary driver's controller.
	 */
	public void drive(XboxController controller) {
		// This was written in ~45 seconds to test the six wheel drive robot.
		// This will be refined over the coming weeks to have cool closed loop
		// control and stuff like that...
		// TODO: Add closed loop control.
		// TODO: Add more robust control system.
		// TODO: Add kinematic expressions.
		// TODO: Add autonomous support.
		// TODO: Add more todos.
		double leftOutput = deadband(-controller.getY(Hand.kLeft), 0.1)
				- Constants.kTurningConstant * deadband(controller.getX(Hand.kRight), 0.1);
		double rightOutput = deadband(-controller.getY(Hand.kLeft), 0.1)
				+ Constants.kTurningConstant * deadband(controller.getX(Hand.kRight), 0.1);

		double[] output = { leftOutput, rightOutput };
		normalize(output);
		
		mLeftMaster.set(ControlMode.PercentOutput, output[0]);
		mRightMaster.set(ControlMode.PercentOutput, output[1]);
	}

	/**
	 * Updates setpoint for autonomous driving.
	 * 
	 * @param distanceInInches
	 *            The distance to drive in inches
	 */
	public void driveSetDistance(double distanceInInches) {
		double naturalUnitDistance = distanceInInches / Constants.kWheelCircumference * Constants.kUnitsPerRotationEnc;
		leftSetpoint = mLeftMaster.getSelectedSensorPosition(0) + naturalUnitDistance;
		rightSetpoint = mRightMaster.getSelectedSensorPosition(0) + naturalUnitDistance;
	}
	
	/**
	 * Uses a PIDF to drive autonomously to setpoint.
	 * 
	 * @see driveSetDistance(double distanceInInches)
	 */
	public void driveDistance() {
		mLeftMaster.set(ControlMode.Position, leftSetpoint);
		mRightMaster.set(ControlMode.Position, rightSetpoint);
	}

	/**
	 * Normalizes a vector.
	 * 
	 * @param array
	 *            Vector to normalize
	 */
	public void normalize(double[] array) {
		double max = Math.abs(array[0]);
		boolean normFlag = max > 1;

		for (int i = 1; i < array.length; i++) {
			if (Math.abs(array[i]) > max) {
				max = Math.abs(array[i]);
				normFlag = max > 1;
			}
		}

		if (normFlag) {
			for (int i = 0; i < array.length; i++) {
				array[i] /= max;
			}
		}
	}

	/**
	 * Applies a deadband (aka minimum value) to a given value.
	 * 
	 * @param value
	 *            Value to apply deadband to.
	 * @param deadband
	 *            Minimum value.
	 * @return value if value is greater than deadband, 0 otherwise.
	 */
	public double deadband(double value, double deadband) {
		return Math.abs(value) > deadband ? value : 0;
	}
}
