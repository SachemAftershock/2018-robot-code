package org.usfirst.frc.team263.robot;

import org.usfirst.frc.team263.robot.Enums.CIMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
	private DoubleSolenoid mSolenoid;
	private CIMode previousMode;

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
		mLeftTalon.setInverted(true);
		mSolenoid = new DoubleSolenoid(0, Constants.kCubeSolFwd, Constants.kCubeSolRev);
		
	}

	// TODO: Institute some closed loop control to ensure similar wheel
	// speeds between each wheel.
	// TODO: Find a better mapping for this on controllers.

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
			mLeftTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed);
			mRightTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed);
			break;
		case eDrop:
			if (previousMode == CIMode.eDrop) {
				return;
			}
			switch (mSolenoid.get()) {
			case kForward:
				mSolenoid.set(Value.kReverse);
				break;
			case kReverse:
				mSolenoid.set(Value.kForward);
				break;
			default:
				mSolenoid.set(Value.kReverse);
			}
			break;
		case eShoot:
			mLeftTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed / 1.1);
			mRightTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed / 1.1);
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
		if (controller.getXButton()) {
			drive(CIMode.eIn);
		} else if (controller.getBButton()) {
			drive(CIMode.eShoot);
		} 
		
		if (controller.getAButton()) {
			drive(CIMode.eDrop);
			previousMode = CIMode.eDrop;
		} else {
			previousMode = CIMode.eStandby;
		}
		
		if (!(controller.getXButton() || controller.getAButton() || controller.getBButton())) {
			drive(CIMode.eStandby);
		}
	}

	/**
	 * Checks if a Cube is currently in the Robot
	 * 
	 * @return True if a Cube is in the mechanism, False otherwise.
	 */
	public boolean isCubeIn() {
		return (mLeftTalon.getOutputCurrent() + mRightTalon.getOutputCurrent()) / 2 > Constants.kCubeCurrentThresh;
	}

	/**
	 * Ejects a Cube until 0.9 seconds have passed.
	 */
	public void autonEjectCube() {
		for (int i = 0; i < 3; i++) {
			drive(CIMode.eShoot);
			Timer.delay(0.3);
		}
		drive(CIMode.eStandby);
	}

	public void autonOpenArm() {
		drive(CIMode.eDrop);
		drive(CIMode.eStandby);
	}

	public void autonIntake() {
		drive(CIMode.eIn);
		drive(CIMode.eDrop);
		drive(CIMode.eIn);
	}
	
	public void autonClose() {
		
		drive(CIMode.eDrop);
	}

}
