package org.usfirst.frc.team263.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Drivebase code for a six-wheel WC-style drive.
 * 
 * @version 0.0.1
 * @author Dan Waxman
 * @since 01-06-2018
 */
public class SWDrive {
	private final double TURN_CONSTANT = 0.5;
	private TalonSRX leftBack, leftFront, rightBack, rightFront;

	/**
	 * Constructor for SWDrive class.
	 * 
	 * @param leftBack
	 *            Left rear motor controller
	 * @param leftFront
	 *            Left front motor controller
	 * @param rightBack
	 *            Right rear motor controller
	 * @param rightFront
	 *            Right front motor controller
	 */
	public SWDrive(TalonSRX leftBack, TalonSRX leftFront, TalonSRX rightBack, TalonSRX rightFront) {
		this.leftBack = leftBack;
		this.leftFront = leftFront;
		this.rightBack = rightBack;
		this.rightFront = rightFront;
	}

	/**
	 * Method to drive robot given controller of primary driver.
	 * 
	 * @param controller
	 *            Primary driver's controller.
	 */
	public void drive(XboxController controller) {
		// This was written in ~45 seconds to test the six wheel drive robot.
		// This will be refined over the coming weeks to have cool closed loop
		// control and stuff like that...
		// TODO: Add closed loop control.
		// TODO: Add more robust control system.
		// TODO: Add kinematic expressions.
		// TODO: Add autonomous support.
		// TODO: Add more todos.
		double leftOutput = -deadband(controller.getY(Hand.kLeft), 0.1)
				+ TURN_CONSTANT * deadband(controller.getX(Hand.kRight), 0.1);
		double rightOutput = deadband(controller.getY(Hand.kLeft), 0.1)
				+ TURN_CONSTANT * deadband(controller.getX(Hand.kRight), 0.1);

		double[] output = { leftOutput, rightOutput };
		normalize(output);

		leftBack.set(ControlMode.PercentOutput, output[0]);
		leftFront.set(ControlMode.PercentOutput, output[0]);
		rightBack.set(ControlMode.PercentOutput, output[1]);
		rightFront.set(ControlMode.PercentOutput, output[1]);
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
}
