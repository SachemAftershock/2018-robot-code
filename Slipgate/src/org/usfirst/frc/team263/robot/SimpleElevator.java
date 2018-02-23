package org.usfirst.frc.team263.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class SimpleElevator {
	
	private static SimpleElevator instance = new SimpleElevator();
	TalonSRX mElevatorTalon;
	
	private SimpleElevator() {
		mElevatorTalon = new TalonSRX(Constants.kElevatorTalon);
	}
	
	public static SimpleElevator getInstance() {
		return instance;
	}
	
	public void drive(XboxController controller) {
		mElevatorTalon.set(ControlMode.PercentOutput, -controller.getY(Hand.kLeft));
	}
	
	private double deadband(double value, double deadband) {
		return Math.abs(value) > deadband ? value : 0;
	}
}
