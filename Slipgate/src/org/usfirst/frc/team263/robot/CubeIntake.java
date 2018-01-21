package org.usfirst.frc.team263.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DoubleSolenoid;

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
	private DigitalInput mLeftLimitSwitch, mRightLimitSwitch;
	// private DoubleSolenoid mCubeSolenoid;

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
		mLeftLimitSwitch = new DigitalInput(Constants.kCubeLeftLimitSwitch);
		mRightLimitSwitch = new DigitalInput(Constants.kCubeRightLimitSwitch);
		// mCubeSolenoid = new DoubleSolenoid(Constants.kRightCubeSolenoid,
		// Constants.kLeftCubeSolenoid);
	}

	/**
	 * Drives the CubeIntake.
	 * 
	 * @param controller
	 *            Controller for CubeIntake instructions.
	 */
	public void drive(XboxController pDriver) {
		// TODO: Institute some closed loop control to ensure similar wheel
		// speeds between each wheel.
		// TODO: Find a better mapping for this on controllers.
		// TODO: Autonomously run wheels using either computer vision, distance
		// reading, or other approach TBD.
		// TODO: Implement DoubleSolenoid 
		
		/*
		 * Stops motors when either limit switch is activated
		 * Allows cube to be sent out via X Button
		 * 
		 */
		while ((mLeftLimitSwitch.get() || mRightLimitSwitch.get()) && !pDriver.getXButton()) { 
			mLeftTalon.set(ControlMode.PercentOutput, 0);
			mRightTalon.set(ControlMode.PercentOutput, 0);

		}
		/*
		 * X Button sends cube out
		 * B Button brings cube in
		 */
		if (pDriver.getXButton()) {
			mRightTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed);
			// mCubeSolenoid.set(DoubleSolenoid.Value.kForward);
			mLeftTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed);
			// mCubeSolenoid.set(DoubleSolenoid.Value.kForward);
		} else if (pDriver.getBButton()) {
			mRightTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed);
			// mCubeSolenoid.set(DoubleSolenoid.Value.kForward);
			mLeftTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed);
			// mCubeSolenoid.set(DoubleSolenoid.Value.kForward);
		} else {
			//mCubeSolenoid.set(DoubleSolenoid.Value.kReverse);
			//mCubeSolenoid.set(DoubleSolenoid.Value.kReverse);
			mRightTalon.set(ControlMode.PercentOutput, 0);
			mLeftTalon.set(ControlMode.PercentOutput, 0);
		}
		// System.out.println("R:" + rightLimitSwitch.get());
		// System.out.println("L:" + leftLimitSwitch.get());
		// System.out.println("");
	}

}
