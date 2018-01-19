package org.usfirst.frc.team263.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Subsystem class for the cube intake.
 * 
 * @author Shreyas Prasad
 * @author Jenn Seibert
 * @author Dan Waxman
 */
public class CubeIntake {
	private static CubeIntake mInstance = new CubeIntake();
	private TalonSRX mLeftTalon, mRightTalon;

	/**
	 * Gets instance of singleton CubeIntake.
	 * 
	 * @return Gets the one instance of CubeIntake.
	 */
	public static CubeIntake getInstance() {
		return mInstance;
	}

	/**
	 * Constructor for CubeIntake class.
	 */
	private CubeIntake() {
		mLeftTalon = new TalonSRX(Constants.kLeftCubeWheel);
		mRightTalon = new TalonSRX(Constants.kRightCubeWheel);
	}

	/**
	 * Drives the CubeIntake.
	 * 
	 * @param controller
	 *            Controller for CubeIntake instructions.
	 */
	public void drive(XboxController controller) {
		// TODO: Institute some closed loop control to ensure similar wheel
		// speeds between each wheel.
		// TODO: Find a better mapping for this on controllers.
		// TODO: Autonomously run wheels using either computer vision, distance
		// reading, or other approach TBD.
		if (controller.getAButtonPressed()) {
			mRightTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed / 2);
			mLeftTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed / 2);
		} else if (controller.getXButtonPressed()) {
			mRightTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed);
			mLeftTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed);
		} else if (controller.getYButtonPressed()) {
			mRightTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed);
			mLeftTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed);
		} else if (controller.getBButtonPressed()) {
			mRightTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed / 2);
			mLeftTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed / 2);
		} else {
			mRightTalon.set(ControlMode.PercentOutput, 0);
			mLeftTalon.set(ControlMode.PercentOutput, 0);
		}
	}

}
