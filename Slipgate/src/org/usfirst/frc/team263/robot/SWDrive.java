package org.usfirst.frc.team263.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Drivebase code for a six-wheel WC-style drive.
 * 
 * @version 0.0.2
 * @author Dan Waxman
 * @since 01-06-2018
 */
public class SWDrive {
	private DriveMode mDriveMode;
	private DriveMode mPreviousDriveMode;
	private static SWDrive mInstance = new SWDrive();
	private TalonSRX mLeftMaster, mLeftSlave, mRightMaster, mRightSlave;
	private AHRS mNavX;
	private double mLeftSetpoint, mRightSetpoint;
	private double mTheta;
	private Direction mCubeAssistDirection;
	private static GearingMode mGearingMode;
	private GearingMode mPreviousGearingMode;
	private Solenoid mLeftSolenoid, mRightSolenoid;

	public static enum Direction {
		eClockwise, eCounterclockwise
	}

	public static enum GearingMode {
		eLowGear, eHighGear
	}

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
		mDriveMode = DriveMode.eOpenLoop;
		mPreviousDriveMode = DriveMode.eOpenLoop;
		mGearingMode = GearingMode.eLowGear;
		mPreviousGearingMode = GearingMode.eLowGear;
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

		mLeftSlave = new TalonSRX(Constants.kLeftSlaveDrivePort);
		mLeftSlave.setInverted(false);
		mLeftSlave.setNeutralMode(NeutralMode.Brake);
		mLeftSlave.follow(mLeftMaster);

		mRightMaster = new TalonSRX(Constants.kRightMasterDrivePort);
		mRightMaster.setNeutralMode(NeutralMode.Brake);
		mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		mRightMaster.setSelectedSensorPosition(0, 0, 0);

		mRightSlave = new TalonSRX(Constants.kRightSlaveDrivePort);
		mRightSlave.setInverted(false);
		mRightSlave.setNeutralMode(NeutralMode.Brake);
		mRightSlave.follow(mRightMaster);
		
		mLeftSolenoid = new Solenoid(Constants.kLeftDriveSolenoidPort);
		mRightSolenoid = new Solenoid(Constants.kRightDriveSolenoidPort);

		setLowGear();
		configureClosedLoop();

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
			if (mDriveMode == DriveMode.eOpenLoop) {
				double leftOutput = deadband(-controller.getY(Hand.kLeft), 0.1)
						+ Constants.kTurningConstant[mGearingMode.ordinal()]
								* deadband(controller.getX(Hand.kRight), 0.1);
				double rightOutput = deadband(-controller.getY(Hand.kLeft), 0.1)
						- Constants.kTurningConstant[mGearingMode.ordinal()]
								* deadband(controller.getX(Hand.kRight), 0.1);

				double[] output = { leftOutput, rightOutput };
				normalize(output);

				mLeftMaster.set(ControlMode.PercentOutput, output[0]);
				mRightMaster.set(ControlMode.PercentOutput, output[1]);
			} else if (mDriveMode == DriveMode.eRotational) {
				// If this is the first loop of the PID, the PID must be
				// initalized.
				if (mPreviousDriveMode != DriveMode.eRotational || mPreviousGearingMode != mGearingMode) {
					PidController.initRotationalPid(Constants.kDriveRKp[mGearingMode.ordinal()],
							Constants.kDriveRKi[mGearingMode.ordinal()], Constants.kDriveRKd[mGearingMode.ordinal()],
							Constants.kDriveRKf[mGearingMode.ordinal()], mTheta);
				}
				double leftOutput = -PidController.getPidOutput();
				double rightOutput = PidController.getPidOutput();

				double[] output = { leftOutput, rightOutput };
				normalize(output);

				mLeftMaster.set(ControlMode.PercentOutput, output[0]);
				mRightMaster.set(ControlMode.PercentOutput, output[1]);
			} else if (mDriveMode == DriveMode.eLinear) {
				// TODO: add motion profiling to linear movement.
				// Using PIDF with encoders right now to drive directly to the
				// setpoints.
				mLeftMaster.set(ControlMode.Position, mLeftSetpoint);
				mRightMaster.set(ControlMode.Position, mRightSetpoint);
			} else if (mDriveMode == DriveMode.eCubeAssist) {
				// TODO: add distance information. This can be done in the
				// future after we decide on where the Limelight is mounted.
				if (Limelight.isTarget()) {
					// If this is the first loop of the PID, the PID must be
					// initalized.
					if (mPreviousDriveMode != DriveMode.eRotational || mPreviousGearingMode != mGearingMode) {
						PidController.initRotationalPid(Constants.kDriveRKp[mGearingMode.ordinal()],
								Constants.kDriveRKi[mGearingMode.ordinal()],
								Constants.kDriveRKd[mGearingMode.ordinal()],
								Constants.kDriveRKf[mGearingMode.ordinal()], mNavX.getYaw() + Limelight.getTx());
					}
					PidController.updateSP(mNavX.getYaw() + Limelight.getTx());

					double leftOutput = 0;
					double rightOutput = 0;

					if (PidController.withinEpsilon() && Limelight.getTa() < 25) {
						leftOutput = Constants.kCubeSeekSpeed[mGearingMode.ordinal()];
						rightOutput = Constants.kCubeSeekSpeed[mGearingMode.ordinal()];
					} else {
						leftOutput = -PidController.getPidOutput();
						rightOutput = PidController.getPidOutput();
					}

					double[] output = { leftOutput, rightOutput };
					normalize(output);

					mLeftMaster.set(ControlMode.PercentOutput, output[0]);
					mRightMaster.set(ControlMode.PercentOutput, output[1]);
				} else {
					if (!ControllerRumble.exists) {
						(new ControllerRumble(controller, 2)).start();
					}

					if (mCubeAssistDirection == Direction.eClockwise) {
						mLeftMaster.set(ControlMode.PercentOutput, Constants.kCubeSeekSpeed[mGearingMode.ordinal()]);
						mRightMaster.set(ControlMode.PercentOutput, -Constants.kCubeSeekSpeed[mGearingMode.ordinal()]);
					} else {
						mLeftMaster.set(ControlMode.PercentOutput, -Constants.kCubeSeekSpeed[mGearingMode.ordinal()]);
						mRightMaster.set(ControlMode.PercentOutput, Constants.kCubeSeekSpeed[mGearingMode.ordinal()]);
					}
				}
			}

			// Set mode to previous mode for SM purposes.
			mPreviousDriveMode = mDriveMode;
			mPreviousGearingMode = mGearingMode;
		}
	}

	/**
	 * Set driving mode to open loop.
	 */
	public void setOpenLoop() {
		mDriveMode = DriveMode.eOpenLoop;
	}

	/**
	 * Configures closed loop variables when gearing modes are changed.
	 */
	private void configureClosedLoop() {
		int index = mGearingMode.ordinal();

		mLeftMaster.config_kP(0, Constants.kDriveKp[index], 0);
		mLeftMaster.config_kI(0, Constants.kDriveKi[index], 0);
		mLeftMaster.config_kD(0, Constants.kDriveKd[index], 0);
		mLeftMaster.config_kF(0, Constants.kDriveKf[index], 0);
		mLeftMaster.configMotionAcceleration(Constants.kDriveAccel[index], 0);
		mLeftMaster.configMotionCruiseVelocity(Constants.kDriveCruiseVelocity[index], 0);
		mLeftMaster.config_IntegralZone(0, Constants.kDriveIZone[index], 0);
		mLeftMaster.configClosedloopRamp(0, Constants.kDriveRampRate[index]);
		mLeftMaster.configOpenloopRamp(0, Constants.kDriveRampRate[index]);
		mLeftMaster.configAllowableClosedloopError(0, Constants.kDriveError[index], 0);

		mRightMaster.config_kP(0, Constants.kDriveKp[index], 0);
		mRightMaster.config_kI(0, Constants.kDriveKi[index], 0);
		mRightMaster.config_kD(0, Constants.kDriveKd[index], 0);
		mRightMaster.config_kF(0, Constants.kDriveKf[index], 0);
		mRightMaster.configMotionAcceleration(Constants.kDriveAccel[index], 0);
		mRightMaster.configMotionCruiseVelocity(Constants.kDriveCruiseVelocity[index], 0);
		mRightMaster.config_IntegralZone(0, Constants.kDriveIZone[index], 0);
		mRightMaster.configClosedloopRamp(0, Constants.kDriveRampRate[index]);
		mRightMaster.configOpenloopRamp(0, Constants.kDriveRampRate[index]);
		mRightMaster.configAllowableClosedloopError(0, Constants.kDriveError[index], 0);
	}

	/**
	 * Sets the drivetrain to low gear.
	 */
	public void setLowGear() {
		setGearingMode(GearingMode.eLowGear);
	}

	/**
	 * Sets the drivetrain to high gear.
	 */
	public void setHighGear() {
		setGearingMode(GearingMode.eHighGear);
	}

	/**
	 * Sets the drivetrain to a given shift mode.
	 * 
	 * @param mode
	 *            Mode to shift to.
	 */
	public void setGearingMode(GearingMode mode) {
		if (mode == GearingMode.eHighGear) {
			mLeftSolenoid.set(true);
			mRightSolenoid.set(true);
		}
		if (mode == GearingMode.eLowGear) {
			mLeftSolenoid.set(false);
			mRightSolenoid.set(false);
		}
		mGearingMode = mode;
		configureClosedLoop();
	}

	/**
	 * Set driving mode to assisted cube seeking.
	 */
	public void setCubeAssist(Direction direction) {
		mCubeAssistDirection = direction;
		mDriveMode = DriveMode.eCubeAssist;
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
		mDriveMode = DriveMode.eLinear;
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
		mDriveMode = DriveMode.eRotational;
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

		public static double getCurrentError() {
			return error;
		}

		/**
		 * Gets the current setpoint of the PID controller.
		 * 
		 * @return The setpoint the PID is trying to achieve.
		 */
		public static double getSP() {
			return setPoint;
		}

		/**
		 * Sees if the controller is in a state of allowable error.
		 * 
		 * @return True if controller is within allowable error, false
		 *         otherwise.
		 */
		public static boolean withinEpsilon() {
			error = rotationalError(SWDrive.getInstance().mNavX.getYaw(), setPoint);
			return Math.abs(error) <= Constants.kDriveREpsilon[mGearingMode.ordinal()];
		}

		/**
		 * Updates setpoint.
		 * 
		 * @param spPrime
		 *            New setpoint.
		 */
		public static void updateSP(double spPrime) {
			setPoint = spPrime;
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
				error = Math.abs(error) > Constants.kDriveREpsilon[mGearingMode.ordinal()] ? error : 0;
			}
			integral += error;
			double u = Kp * error + Ki * integral + Kd * (error - previousError);
			previousError = error;
			if (Math.abs(u) < Constants.kDriveRStaticFr[mGearingMode.ordinal()]) {
				u += Math.signum(u) * Constants.kDriveRStaticFr[mGearingMode.ordinal()];
			}
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
