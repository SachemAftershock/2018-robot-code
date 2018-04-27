package org.usfirst.frc.team263.robot;

import org.usfirst.frc.team263.robot.Limelight.CameraMode;
import org.usfirst.frc.team263.robot.Enums.Direction;
import org.usfirst.frc.team263.robot.Enums.DriveMode;
import org.usfirst.frc.team263.robot.Enums.GearingMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

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
	private TalonSRX mLeftMaster, mRightMaster;
	private VictorSPX mLeftSlave, mRightSlave;
	private AHRS mNavX;
	private double mLeftSetpoint, mRightSetpoint;
	private double mTheta, mVelocityRatio;
	private Direction mCubeAssistDirection;
	private static GearingMode mGearingMode;
	private GearingMode mPreviousGearingMode;
	private Solenoid mSolenoid;
	private MagicElevator mElev;
	private double leftOutputPrev, rightOutputPrev;
	private boolean mIsSetpointReached, startPressed;
	boolean f;
	int osci;

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
		f = false;
		startPressed = false;
		mVelocityRatio = 0;

		// Initialize all master and slave motors.
		mLeftMaster = new TalonSRX(Constants.kLeftMasterDrivePort);
		mLeftMaster.setNeutralMode(NeutralMode.Brake);
		mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		mLeftMaster.setSelectedSensorPosition(0, 0, 0);
		mLeftMaster.setSensorPhase(false);
		mLeftMaster.setInverted(false);

		mLeftSlave = new VictorSPX(Constants.kLeftSlaveDrivePort);
		mLeftSlave.setInverted(false);
		mLeftSlave.setNeutralMode(NeutralMode.Brake);
		mLeftSlave.follow(mLeftMaster);

		mRightMaster = new TalonSRX(Constants.kRightMasterDrivePort);
		mRightMaster.setNeutralMode(NeutralMode.Brake);
		mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		mRightMaster.setSelectedSensorPosition(0, 0, 0);
		mRightMaster.setInverted(true);

		mRightSlave = new VictorSPX(Constants.kRightSlaveDrivePort);
		mRightSlave.setInverted(true);
		mRightSlave.setNeutralMode(NeutralMode.Brake);
		mRightSlave.follow(mRightMaster);

		mSolenoid = new Solenoid(1, Constants.kDriveSolenoidPort);

		setLowGear();
		configureClosedLoop();

		mIsSetpointReached = true;

		// Initialize NavX in MXP port.
		mNavX = new AHRS(SPI.Port.kMXP);
		Timer.delay(1);
		System.out.println("NavX calibration: " + mNavX.isCalibrating());
		
		mElev = MagicElevator.getInstance();
	}

	/**
	 * Drive robot in any DriveMode.
	 * 
	 * @param leftY
	 *            y component of open loop control
	 * @param rightX
	 *            x component of open loop control
	 */
	public void drive(double leftY, double rightX) {
		synchronized (this) {
			if (mDriveMode == DriveMode.eOpenLoop) {
				// Determine normal inputs
				double leftOutput = deadband(leftY, 0.1)
						+ Constants.kTurningConstant[mGearingMode.ordinal()] * deadband(rightX, 0.1);
				double rightOutput = deadband(leftY, 0.1)
						- Constants.kTurningConstant[mGearingMode.ordinal()] * deadband(rightX, 0.1);
				
				
				// Create logs of previous outputs for acceleration purposes
				// Report proposed acceleration as zero if elevator is low
				if (mElev.getHeight() < Constants.kAccelLimHeight) {
					leftOutputPrev = leftOutput;
					rightOutputPrev = rightOutput;
				}
				
				
				// Limit acceleration per loop
				if (Math.abs(leftOutput - leftOutputPrev) > Constants.kAccelPercentPerLoop) {
					leftOutput =  leftOutputPrev + Math.signum(leftOutput - leftOutputPrev) * Constants.kAccelPercentPerLoop;
				}
				if (Math.abs(rightOutput - rightOutputPrev) > Constants.kAccelPercentPerLoop) {
					rightOutput = rightOutputPrev + Math.signum(rightOutput - rightOutputPrev) * Constants.kAccelPercentPerLoop;
				}			
				
				double[] output = { leftOutput, rightOutput };
				normalize(output);

				mLeftMaster.set(ControlMode.PercentOutput, output[0]);
				mRightMaster.set(ControlMode.PercentOutput, output[1]);
				
				leftOutputPrev = leftOutput;
				rightOutputPrev = rightOutput;

				mIsSetpointReached = true;
			} else if (mDriveMode == DriveMode.eRotational) {
				// If this is the first loop of the PID, the PID must be
				// initalized.
				if (mPreviousDriveMode != DriveMode.eRotational || mPreviousGearingMode != mGearingMode
						|| mTheta != PidController.setPoint) {
					PidController.initRotationalPid(Constants.kDriveRKp[mGearingMode.ordinal()],
							Constants.kDriveRKi[mGearingMode.ordinal()], Constants.kDriveRKd[mGearingMode.ordinal()],
							Constants.kDriveRKf[mGearingMode.ordinal()], mTheta);
				}
				double u = PidController.getPidOutput();
				double leftOutput = -u;
				double rightOutput = u;

				double[] output = { leftOutput, rightOutput };
				normalize(output);

				mLeftMaster.set(ControlMode.PercentOutput, output[0]);
				mRightMaster.set(ControlMode.PercentOutput, output[1]);

				mIsSetpointReached = PidController.withinEpsilon();
			} else if (mDriveMode == DriveMode.eLinear) {
				// TODO: add motion profiling to linear movement.
				// Using PIDF with encoders right now to drive directly to the
				// setpoints.
				mLeftMaster.set(ControlMode.Position, mLeftSetpoint);
				mRightMaster.set(ControlMode.Position, mRightSetpoint);

				mIsSetpointReached = (Math.abs(mLeftMaster.getSelectedSensorPosition(0)
						- mLeftSetpoint) <= Constants.kDriveError[mGearingMode.ordinal()])
						&& (Math.abs(mRightMaster.getSelectedSensorPosition(0)
								- mRightSetpoint) <= Constants.kDriveError[mGearingMode.ordinal()]);
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

					if (PidController.withinEpsilon()) {
						System.out.println("Forward phase");
						leftOutput = 0.3;
						rightOutput = 0.3;
						Timer.delay(0.2);
						mIsSetpointReached = true;
					} else {
						f = false;
						leftOutput = -PidController.getPidOutput();
						rightOutput = PidController.getPidOutput();
					}

					double[] output = { leftOutput, rightOutput };
					normalize(output);

					mLeftMaster.set(ControlMode.PercentOutput, output[0]);
					mRightMaster.set(ControlMode.PercentOutput, output[1]);

					mIsSetpointReached = PidController.withinEpsilon();
				} else {
					System.out.println("Searching...");
					// TODO: Add back controller feedback here.
					if (mCubeAssistDirection == Direction.eClockwise) {
						mLeftMaster.set(ControlMode.PercentOutput, Constants.kCubeSeekSpeed[mGearingMode.ordinal()]);
						mRightMaster.set(ControlMode.PercentOutput, -Constants.kCubeSeekSpeed[mGearingMode.ordinal()]);
					} else {
						mLeftMaster.set(ControlMode.PercentOutput, -Constants.kCubeSeekSpeed[mGearingMode.ordinal()]);
						mRightMaster.set(ControlMode.PercentOutput, Constants.kCubeSeekSpeed[mGearingMode.ordinal()]);
					}

					mIsSetpointReached |= false;
				}
			} else if (mDriveMode == DriveMode.eCurve) {
				// If this is the first loop of the PID, the PID must be
				// initalized.
				if (mPreviousDriveMode != DriveMode.eCurve || mPreviousGearingMode != mGearingMode) {
					PidController.initRotationalPid(Constants.kDriveCKp[mGearingMode.ordinal()],
							Constants.kDriveCKi[mGearingMode.ordinal()], Constants.kDriveCKd[mGearingMode.ordinal()],
							Constants.kDriveCKf[mGearingMode.ordinal()], mTheta);
				}
				double leftOutput = mVelocityRatio * PidController.getPidOutput();
				double rightOutput = PidController.getPidOutput();

				double[] output = { leftOutput, rightOutput };
				normalize(output);

				mLeftMaster.set(ControlMode.PercentOutput, -output[0]);
				mRightMaster.set(ControlMode.PercentOutput, -output[1]);
			}

			// Set mode to previous mode for SM purposes.
			mPreviousDriveMode = mDriveMode;
			mPreviousGearingMode = mGearingMode;
		}
	}

	/**
	 * Method to drive robot given controller of primary driver.
	 * 
	 * @param controller
	 *            Primary driver's controller.
	 */
	public void drive(XboxController controller) {
		//System.out.println("Right Master: " + mRightMaster.getSelectedSensorPosition(0) + ", Left Master: " +mLeftMaster.getSelectedSensorPosition(0));
		double leftStick = -controller.getY(Hand.kLeft);
		double rightStick = controller.getX(Hand.kRight);

		// if tilt on and angle within [thresh,45] [thresh,45] -> [.1, .4]
		
		//TODO: check if this should be absolute valued
		//			- i remember it working in gym but logically seems incorrect
		//			- should only the comparison be absolute valued?
		float pitch = Math.abs(mNavX.getPitch());
		if (pitch > Constants.kTiltThresh && pitch < 45) {
			double slope = (0.4 - 0.1) / (45 - Constants.kTiltThresh);
			double correctionOffset = slope * pitch - Constants.kTiltThresh;
			double[] tmp = { leftStick + correctionOffset, rightStick + correctionOffset };
			normalize(tmp);
			leftStick = tmp[0];
			rightStick = tmp[1];
		}

		if (deadband(controller.getTriggerAxis(Hand.kLeft), 0.5) == 0) {
			drive(leftStick, rightStick);
		} else {
			drive(-Constants.kDriveMultiplier * leftStick, Constants.kDriveMultiplier * rightStick);
		}
	}

	/**
	 * Drive robot without open loop control.
	 */
	public void drive() {
		drive(0, 0);
	}

	/**
	 * Returns information on whether closed loop setpoint is reached.
	 * 
	 * @return true if setpoint has been reached, false otherwise.
	 */
	public boolean isSetpointReached() {
		return mIsSetpointReached;
	}

	/**
	 * Set driving mode to open loop.
	 */
	public void setOpenLoop() {
		mDriveMode = DriveMode.eOpenLoop;
	}

	public double getYaw() {
		return mNavX.getYaw();
	}

	/**
	 * Sets up curved closed loop control.
	 * 
	 * @param r
	 *            Signed distance to ICC.
	 * @param theta
	 *            Angular offset to travel.
	 */
	public void setCurveControl(double r, double theta) {
		mTheta = theta;
		mVelocityRatio = Kinematics.getVelocityRatio(r, Constants.kDriveWheelLength);
		mDriveMode = DriveMode.eCurve;
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
			mSolenoid.set(true);
		}
		if (mode == GearingMode.eLowGear) {
			mSolenoid.set(false);
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
		Limelight.setCameraMode(CameraMode.eVision);
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
		mIsSetpointReached = false;
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
		mIsSetpointReached = false;
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
		 * @return True if controller is within allowable error, false otherwise.
		 */
		public static boolean withinEpsilon() {
			error = rotationalError(SWDrive.getInstance().mNavX.getYaw(), setPoint);
			if (SWDrive.getInstance().mDriveMode == DriveMode.eCubeAssist) {
				return Math.abs(error) <= Constants.kDriveREpsilon[mGearingMode.ordinal()] * 5;
			}
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
				if (SWDrive.getInstance().mDriveMode == DriveMode.eCubeAssist) {
					error = Math.abs(error) > 5 * Constants.kDriveREpsilon[mGearingMode.ordinal()] ? error : 0;

				}
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

	public double getPid() {
		return PidController.getPidOutput();
	}
}
