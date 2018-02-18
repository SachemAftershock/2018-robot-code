package org.usfirst.frc.team263.robot;

import org.usfirst.frc.team263.robot.Enums.ElevatorPosition;
import org.usfirst.frc.team263.robot.Enums.LEDMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Controller for elevator system on robot.
 * 
 * @version 0.0.1
 * @author Rohan Bapat
 * @since 02-18-2018
 */
public class MagicElevator {

	private static MagicElevator instance = new MagicElevator();

	private boolean upPressed, downPressed, L3Pressed, nudgeUpPressed, nudgeDownPressed, running;
	private ElevatorPosition targetLevel, elevatorLevel;
	private TalonSRX mElevatorTalon;
	private DigitalOutput override;
	private int[] encoderLevels;
	private int currentCount;

	/**
	 * Constructor for magic elevator class
	 */
	private MagicElevator() {
		// configure TalonSRX for closed-loop control
		mElevatorTalon = new TalonSRX(Constants.kElevatorTalon);
		mElevatorTalon.setNeutralMode(NeutralMode.Brake);
		mElevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		mElevatorTalon.setSensorPhase(true);
		mElevatorTalon.config_kP(0, Constants.kElevatorKp, 0);
		mElevatorTalon.config_kI(0, Constants.kElevatorKi, 0);
		mElevatorTalon.config_kD(0, Constants.kElevatorKd, 0);
		mElevatorTalon.config_kF(0, Constants.kElevatorKf, 0);
		mElevatorTalon.setSelectedSensorPosition(0, Constants.kInitialCount, 0); // TODO: CHECK DOCUMENTATION

		// elevator and encoder levels
		targetLevel = ElevatorPosition.kInitial;
		elevatorLevel = ElevatorPosition.kInitial;

		// TODO: replace with actual values taken from open-loop control on elevator
		encoderLevels = new int[] { -1, 118, 254, 263, 353, 1156 };
		currentCount = Constants.kInitialCount;

		// booleans for button presses and profile availablility
		upPressed = false;
		downPressed = false;
		L3Pressed = false;
		nudgeUpPressed = false;
		nudgeDownPressed = false;

		// Manual override for kill switch
		override = new DigitalOutput(Constants.kOverride);
	}

	/**
	 * Gets instance of singleton MagicElevator.
	 * 
	 * @return Gets the one instance of MagicElevator.
	 */
	public static MagicElevator getInstance() {
		return instance;
	}

	/**
	 * Method to drive robot given controller of secondary driver.
	 * 
	 * @param controller
	 *            Secondary driver's controller.
	 */
	public void drive(XboxController controller) {
		if (deadband(controller.getTriggerAxis(Hand.kLeft), 0.5) != 0) {
			clearEverything();
			mElevatorTalon.set(ControlMode.PercentOutput, deadband(-controller.getY(Hand.kLeft), 0.1));
		} else if (controller.getBumper(Hand.kRight) && !upPressed) {
			targetLevel = ElevatorPosition.values()[Math.min(targetLevel.ordinal() + 1, encoderLevels.length - 1)];
		} else if (controller.getBumper(Hand.kLeft) && !downPressed) {
			targetLevel = ElevatorPosition.values()[Math.max(targetLevel.ordinal() - 1, 1)];
		} else if (controller.getBumper(Hand.kLeft) && controller.getBumper(Hand.kRight)) {
			clearEverything();
			targetLevel = elevatorLevel;
		} else if (controller.getPOV() == 0 && !nudgeUpPressed) {
			encoderLevels[elevatorLevel.ordinal()] += 20;
			executeHead();
		} else if (controller.getPOV() == 180 && !nudgeDownPressed) {
			encoderLevels[elevatorLevel.ordinal()] -= 20;
			executeHead();
		} else if (controller.getStickButton(Hand.kLeft) && !L3Pressed) {
			executeHead();
		}

		if (controller.getStartButton()) {
			override.set(true);
			LEDStrip.sendColor(LEDMode.eOverrideToggle);
		}
		
		if(running && !atTarget()) {
			mElevatorTalon.set(ControlMode.MotionMagic, encoderLevels[targetLevel.ordinal()]);
		}
		
		if (getDelta(elevatorLevel) > Constants.kElevatorThreshhold) {
			for (ElevatorPosition el : ElevatorPosition.values()) {
				if (getDelta(el) < Constants.kElevatorThreshhold) {
					elevatorLevel = el;
					running = false;
				}
			}
		}
		

		upPressed = controller.getBumper(Hand.kRight);
		downPressed = controller.getBumper(Hand.kLeft);
		L3Pressed = controller.getStickButton(Hand.kLeft);
		nudgeUpPressed = controller.getPOV() == 0;
		nudgeDownPressed = controller.getPOV() == 180;
	}
	
	/**
	 * Gets distance between given elevator position and actual elevator position in
	 * natural units
	 * 
	 * @param position
	 *            elevator position to find delta
	 * 
	 * @return Gets distance between assumed elevator position and actual elevator
	 *         position in natural units
	 */
	private int getDelta(ElevatorPosition position) {
		return Math.abs(currentCount - encoderLevels[position.ordinal()]);
	}
	
	private boolean atTarget() {
		return elevatorLevel == targetLevel && getDelta(elevatorLevel) < Constants.kElevatorThreshhold;
	}
	/**
	 * Executes the command at the head of the trajectory queue.
	 * 
	 * Not the Maximilien Robespierre style though.
	 */
	private void executeHead() {
		if (!atTarget()) {
			running = true;
		}
	}

	/**
	 * Resets target level
	 */
	private void clearEverything() {
		targetLevel = elevatorLevel;
	}

	/**
	 * Applies a deadband to a given value.
	 * 
	 * @param value
	 *            Value to apply deadband to.
	 * @param deadband
	 *            Minimum value.
	 * @return value if value is greater than deadband, 0 otherwise.
	 */
	private double deadband(double value, double deadband) {
		return Math.abs(value) > deadband ? value : 0.0;
	}
}
