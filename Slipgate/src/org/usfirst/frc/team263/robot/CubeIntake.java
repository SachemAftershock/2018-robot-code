package org.usfirst.frc.team263.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.XboxController;

public class CubeIntake {
	private static CubeIntake mInstance = new CubeIntake();
	private TalonSRX mLeftTalon, mRightTalon;

	public static CubeIntake getInstance() {
		return mInstance;
	}

	private CubeIntake() {
		mLeftTalon = new TalonSRX(Constants.kLeftCubeWheel);
		mRightTalon = new TalonSRX(Constants.kRightCubeWheel);

	}

	public void drive(XboxController pDriver) {
		if (pDriver.getAButtonPressed()) {
			mRightTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed / 2);
			mLeftTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed / 2);
		}
		if (pDriver.getXButtonPressed()) {
			mRightTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed);
			mLeftTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed);
		}
		if (pDriver.getYButtonPressed()) {
			mRightTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed);
			mLeftTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed);
		}
		if (pDriver.getBButtonPressed()) {
			mRightTalon.set(ControlMode.PercentOutput, -Constants.kCubeWheelSpeed / 2);
			mLeftTalon.set(ControlMode.PercentOutput, Constants.kCubeWheelSpeed / 2);
		}
		else {
			mRightTalon.set(ControlMode.PercentOutput, 0);
			mLeftTalon.set(ControlMode.PercentOutput, 0);
		}
	}

}
