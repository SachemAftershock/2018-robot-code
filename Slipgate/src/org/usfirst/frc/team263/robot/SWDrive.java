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
	private DriveMode mMode;
	private DriveMode mPreviousMode;
	private static SWDrive mInstance = new SWDrive();
	private TalonSRX mLeftMaster, mLeftSlave, mRightMaster, mRightSlave;
	private AHRS mNavX;
	private double mLeftSetpoint, mRightSetpoint;
	private double mTheta;

	/**
	 * Gets instance of singleton SWDrive.
	 * 
	 * @return Gets the one instance of SWDrive.
	 */
	public static SWDrive getInstance() {
		return mInstance;
	}

	/**
	 * Constructor for SWDrive class.
	 */
	private SWDrive() {
		// Set drive controls to open loop by default and initialize all
		// setpoints to 0.
		mMode = DriveMode.eOpenLoop;
		mPreviousMode = DriveMode.eOpenLoop;
		mLeftSetpoint = 0;
		mRightSetpoint = 0;
		mTheta = 0;

		// Initialize all master and slave motors.
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
		mLeftMaster.configMotionAcceleration(Constants.kDriveAccel, 0);
		mLeftMaster.configMotionCruiseVelocity(Constants.kDriveCruiseVelocity, 0);
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
		mLeftMaster.configMotionAcceleration(Constants.kDriveAccel, 0);
		mLeftMaster.configMotionCruiseVelocity(Constants.kDriveCruiseVelocity, 0);
		mRightMaster.config_IntegralZone(0, Constants.kDriveIZone, 0);
		mRightMaster.configClosedloopRamp(0, Constants.kDriveRampRate);
		mRightMaster.configOpenloopRamp(0, Constants.kDriveRampRate);
		mRightMaster.configAllowableClosedloopError(0, Constants.kDriveError, 0);

		mRightSlave = new TalonSRX(Constants.kRightSlaveDrivePort);
		mRightSlave.setInverted(true);
		mRightSlave.setNeutralMode(NeutralMode.Brake);
		mRightSlave.follow(mRightMaster);

		// Initialize NavX in MXP port.
		mNavX = new AHRS(SPI.Port.kMXP);
	}

	/**
	 * Method to drive robot given controller of primary driver.
	 * 
	 * @param controller
	 *            Primary driver's controller.
	 */
	public void drive(XboxController controller) {
		synchronized (this) {
			if (mMode == DriveMode.eOpenLoop) {
				double leftOutput = deadband(-controller.getY(Hand.kLeft), 0.1)
						- Constants.kTurningConstant * deadband(controller.getX(Hand.kRight), 0.1);
				double rightOutput = deadband(-controller.getY(Hand.kLeft), 0.1)
						+ Constants.kTurningConstant * deadband(controller.getX(Hand.kRight), 0.1);

				double[] output = { leftOutput, rightOutput };
				normalize(output);

				mLeftMaster.set(ControlMode.PercentOutput, output[0]);
				mRightMaster.set(ControlMode.PercentOutput, output[1]);
			} else if (mMode == DriveMode.eRotational) {
				// If this is the first loop of the PID, the PID must be
				// initalized.
				if (mPreviousMode != DriveMode.eRotational) {
					PidController.initRotationalPid(Constants.kDriveRKp, Constants.kDriveRKi, Constants.kDriveRKd,
							Constants.kDriveRKf, mTheta);
				}
				double leftOutput = PidController.getPidOutput();
				double rightOutput = -PidController.getPidOutput();

				double[] output = { leftOutput, rightOutput };
				normalize(output);

				mLeftMaster.set(ControlMode.PercentOutput, output[0]);
				mRightMaster.set(ControlMode.PercentOutput, output[1]);
			} else if (mMode == DriveMode.eLinear) {
				// TODO: add motion profiling to linear movement.
				// Using PIDF with encoders right now to drive directly to the
				// setpoints.
				mLeftMaster.set(ControlMode.Position, mLeftSetpoint);
				mRightMaster.set(ControlMode.Position, mRightSetpoint);
			} else if (mMode == DriveMode.eCubeAssist) {
				// TODO: add distance information. This can be done in the
				// future after we decide on where the Limelight is mounted.
				if (Limelight.isTarget()) {
					// If this is the first loop of the PID, the PID must be
					// initalized.
					if (mPreviousMode != DriveMode.eRotational) {
						PidController.initRotationalPid(Constants.kDriveRKp, Constants.kDriveRKi, Constants.kDriveRKd,
								Constants.kDriveRKf, mNavX.getYaw() - Limelight.getTx());
					}
					double leftOutput = PidController.getPidOutput();
					double rightOutput = -PidController.getPidOutput();

					double[] output = { leftOutput, rightOutput };
					normalize(output);

					mLeftMaster.set(ControlMode.PercentOutput, output[0]);
					mRightMaster.set(ControlMode.PercentOutput, output[1]);
				} else {
					if (!ControllerRumble.exists) {
						(new ControllerRumble(controller, 2)).start();
					}

					double leftOutput = deadband(-controller.getY(Hand.kLeft), 0.1)
							- Constants.kTurningConstant * deadband(controller.getX(Hand.kRight), 0.1);
					double rightOutput = deadband(-controller.getY(Hand.kLeft), 0.1)
							+ Constants.kTurningConstant * deadband(controller.getX(Hand.kRight), 0.1);

					double[] output = { leftOutput, rightOutput };
					normalize(output);

					mLeftMaster.set(ControlMode.PercentOutput, output[0]);
					mRightMaster.set(ControlMode.PercentOutput, output[1]);
				}
			}

			// Set mode to previous mode for SM purposes.
			mPreviousMode = mMode;
		}
	}

	/**
	 * Set driving mode to open loop
	 */
	public void setOpenLoop() {
		mMode = DriveMode.eOpenLoop;
	}

	/**
	 * Updates setpoint for autonomous driving.
	 * 
	 * @param distanceInInches
	 *            The distance to drive in inches
	 */
	public void setLinearDistance(double distanceInInches) {
		double naturalUnitDistance = distanceInInches / Constants.kWheelCircumference * Constants.kUnitsPerRotationEnc;
		mLeftSetpoint = mLeftMaster.getSelectedSensorPosition(0) + naturalUnitDistance;
		mRightSetpoint = mRightMaster.getSelectedSensorPosition(0) + naturalUnitDistance;
		mMode = DriveMode.eLinear;
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
	 * Sets a setpoint for rotational PID.
	 * 
	 * @param theta
	 *            Angle to rotate to in range (-180, 180]
	 */
	public void setRotationTheta(double theta) {
		mTheta = theta;
		mMode = DriveMode.eRotational;
	}

	/**
	 * Zeros navX yaw.
	 */
	public void zeroGyro() {
		mNavX.zeroYaw();
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

	/**
	 * Class for closed loop PID control.
	 * 
	 * @author Dan Waxman
	 */
	private static class PidController {
		private static double Kp, Ki, Kd, Kf, setPoint, error, previousError, integral;
		private static boolean rotation;

		/**
		 * Initiate rotational PID.
		 * 
		 * @param kp
		 *            Constant multiple for P term.
		 * @param ki
		 *            Constant multiple for I term.
		 * @param kd
		 *            Constant multiple for D term.
		 * @param kf
		 *            Constant multiple for F term.
		 * @param setpoint
		 *            Angle to rotate to.
		 */
		public static void initRotationalPid(double kp, double ki, double kd, double kf, double setpoint) {
			Kp = kp;
			Ki = ki;
			Kd = kd;
			Kf = 0;
			rotation = true;
			setPoint = setpoint;
			error = rotationalError(SWDrive.getInstance().mNavX.getYaw(), setPoint);
			previousError = error;
			integral = 0;
		}

		/**
		 * Gets PID Output.
		 * 
		 * @return Gets output of PID controller based upon configured mode and
		 *         constants.
		 */
		public static double getPidOutput() {
			double error = 0;
			if (rotation) {
				error = rotationalError(SWDrive.getInstance().mNavX.getYaw(), setPoint);
				System.out.println(error);
				error = Math.abs(error) > Constants.kDriveREpsilon ? error : 0;
			}
			integral += error;
			double u = Kp * error + Ki * integral + Kd * (error - previousError);
			previousError = error;
			return u;
		}

		/**
		 * Gets rotational error on [-180, 180]
		 * 
		 * @param alpha
		 *            First angle
		 * @param beta
		 *            Second Angle
		 * @return Rotational error
		 */
		private static double rotationalError(double alpha, double beta) {
			double ret = alpha - beta;
			if (ret < -180) {
				ret += 360;
			}
			if (ret > 180) {
				ret -= 360;
			}
			return ret;
		}
	}
}
