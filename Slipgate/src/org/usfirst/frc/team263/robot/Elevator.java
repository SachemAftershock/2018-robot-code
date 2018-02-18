package org.usfirst.frc.team263.robot;

import org.usfirst.frc.team263.robot.Enums.ElevatorPosition;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Controller for elevator system on robot.
 * 
 * @version 0.0.1
 * @author Rohan Bapat
 * @since 01-29-2018
 */
public class Elevator {

	private static Elevator instance = new Elevator();
	TalonSRX mElevatorTalon;

	/**
	 * Constructor for elevator singleton.
	 */
	private Elevator() {
		mElevatorTalon = new TalonSRX(Constants.kElevatorTalon);
		mElevatorTalon.setNeutralMode(NeutralMode.Brake);
	}

	/**
	 * Gets instance of Elevator singleton.
	 * 
	 * @return singleton instance of elevator.
	 */
	public static Elevator getInstance() {
		return instance;
	}

	/**
	 * Method for open loop control of elevator given controller of secondary
	 * driver.
	 * 
	 * @param controller
	 *            Secondary driver's controller.
	 */
	public void drive(XboxController controller) {
		mElevatorTalon.set(ControlMode.PercentOutput, deadband(-controller.getY(Hand.kLeft), 0.1));
	}

	/**
	 * Applies a deadband to a given value.
	 * 
	 * @param value
	 *            Value to apply deadband to.
	 * @param deadband
	 *            Minimum deadband value.
	 * @return value if value's magnitude is greater than deadband, 0 otherwise.
	 */
	private double deadband(double value, double deadband) {
		return Math.abs(value) > deadband ? value : 0.0;
	}

	/**
	 * Sends elevator to given elevator position
	 * 
	 * @param position
	 *            ElevatorPosition enum signalling which position to go to
	 */
	public void sendTo(ElevatorPosition position) {
		// TODO: implement this with all the other closed loop control
	}
}
