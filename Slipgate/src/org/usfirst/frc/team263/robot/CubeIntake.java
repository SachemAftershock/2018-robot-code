package org.usfirst.frc.team263.robot;

import org.usfirst.frc.team263.robot.Enums.CIMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

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
	}

	// TODO: Institute some closed loop control to ensure similar wheel
	// speeds between each wheel.
	// TODO: Find a better mapping for this on controllers.
	// TODO: Autonomously run wheels using either computer vision, distance
	// reading, or other approach TBD.
	// TODO: Implement Solenoids

	/**
	 * Drives CubeIntake subsystem with speed given by parameter
	 * 
	 * @param speed
	 *            Speed of CubeIntake
	 */
	public void drive(CIMode mode) {
		switch (mode) {
		case eStandby:
			mLeftTalon.set(ControlMode.PercentOutput, 0);
			mRightTalon.set(ControlMode.PercentOutput, 0);
			break;
		case eIn:
			// TODO: Implement arms grabbing
			mLeftTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed);
			mRightTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed);
			break;
		case eDrop:
			// TODO: Implement arms dropping
			break;
		case eShoot:
			mLeftTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed);
			mRightTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed);
			break;
		}
	}

	/**
	 * Drives the CubeIntake.
	 *  X brings Cube in
	 *  B sends Cube out
	 * @param controller
	 *            Controller for CubeIntake instructions.
	 */
	public void drive(XboxController controller) {
		if (controller.getXButton() && !isCubeIn()) {
			drive(CIMode.eIn);
		} else if (controller.getBButton()) {
			drive(CIMode.eShoot);
		} else if (controller.getAButton()) {
			drive(CIMode.eDrop);
		} else {
			drive(CIMode.eStandby);
		}
	}

	/**
	 * Checks if a Cube is currently in the Robot
	 * 
	 * @return True if a Cube is in the mechanism, False otherwise.
	 */
	public boolean isCubeIn() {
		return (mLeftLimitSwitch.get() || mRightLimitSwitch.get());
	}

	/**
	 * Ejects a Cube until it is no longer triggering the limit switches or 0.9
	 * seconds are up, whichever comes first.
	 */
	public void autonEjectCube() {
		for (int i = 0; isCubeIn() && i < 3; i++) {
			drive(CIMode.eShoot);
			Timer.delay(0.3);// TODO Figure out minimum time needed to eject
								// Cube
			// Would be optimal to have encoder on here but that doesn't seem
			// feasible. Time is next best option
		}
		drive(CIMode.eStandby);
	}

}
