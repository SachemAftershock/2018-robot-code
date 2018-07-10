package org.usfirst.frc.team263.robot;

import org.usfirst.frc.team263.robot.Enums.ElevatorPosition;
import org.usfirst.frc.team263.robot.Enums.LEDMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Controller for elevator system on robot.
 * 
 * @version 0.0.1
 * @author Rohan Bapat
 * @since 02-18-2018
 */
public class MagicElevator {

	private static MagicElevator instance = new MagicElevator();

	private boolean upPressed, downPressed, L3Pressed, nudgeUpPressed, nudgeDownPressed, running, ltPressed, yPressed,
			startPressed, r3Pressed;
	private ElevatorPosition targetLevel, elevatorLevel;
	private VictorSPX climberVictor;
	private TalonSRX mElevatorTalon;
	private DoubleSolenoid tiltSolenoid, climberSolenoid;;
	private DigitalOutput override;
	private int[] encoderLevels;
	private int currentCount;
	private double timeSinceMotorCommand;
	private double previous;
	private boolean encoderHealthy;

	/**
	 * Constructor for magic elevator class
	 */
	private MagicElevator() {
		// configure TalonSRX for closed-loop control
		mElevatorTalon = new TalonSRX(Constants.kElevatorTalon);
		mElevatorTalon.setNeutralMode(NeutralMode.Brake);
		mElevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		mElevatorTalon.setSensorPhase(false);
		mElevatorTalon.config_kP(0, 3.0, 0);
		mElevatorTalon.config_kI(0, 0.0, 0);
		mElevatorTalon.config_kD(0, 0.0, 0);
		mElevatorTalon.config_kF(0, 0.0, 0);
		mElevatorTalon.setInverted(true);
		mElevatorTalon.configMotionAcceleration(1000, 0);
		mElevatorTalon.configMotionCruiseVelocity(5000, 0);
		mElevatorTalon.config_IntegralZone(0, 200, 0);
		mElevatorTalon.configClosedloopRamp(0, 256);
		mElevatorTalon.configOpenloopRamp(0, 256);
		mElevatorTalon.configAllowableClosedloopError(0, Constants.kElevatorThreshhold, 0);

		mElevatorTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
				LimitSwitchNormal.NormallyOpen, 0);
		mElevatorTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
				LimitSwitchNormal.NormallyOpen, 0);

		climberVictor = new VictorSPX(Constants.kClimberVictor);
		climberVictor.setNeutralMode(NeutralMode.Brake);
		climberSolenoid = new DoubleSolenoid(0, Constants.kClimberSolFwd, Constants.kClimberSolRev);

		// elevator and encoder levels
		targetLevel = ElevatorPosition.kInitial;
		elevatorLevel = ElevatorPosition.kInitial;

		// TODO: replace with actual values taken from open-loop control on elevator
		// elevator { reserved, ground, vault, switch, minScale, midScale, maxScale }
		encoderLevels = new int[] { -1, 0, 263, /*1410*/ 2000, 4000, 4650, 5100 };
		tiltSolenoid = new DoubleSolenoid(0, Constants.kElevatorSolFwd, Constants.kElevatorSolRev);
		currentCount = Constants.kInitialCount;

		// booleans for button presses and profile availablility
		upPressed = false;
		downPressed = false;
		L3Pressed = false;
		nudgeUpPressed = false;
		yPressed = false;
		ltPressed = false;
		nudgeDownPressed = false;
		startPressed = false;
		r3Pressed = false;
		encoderHealthy = true;

		// Manual override for kill switch
		override = new DigitalOutput(Constants.kOverride);

		timeSinceMotorCommand = 0;
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
		currentCount = mElevatorTalon.getSelectedSensorPosition(0);
		System.out.println("ElevatorEncoder: " + currentCount);
		if (deadband(controller.getTriggerAxis(Hand.kLeft), 0.5) != 0) {
			mElevatorTalon.set(ControlMode.PercentOutput, deadband(-controller.getY(Hand.kLeft), 0.1));
			timeSinceMotorCommand = 0;
		} else if (deadband(controller.getTriggerAxis(Hand.kLeft), 0.5) == 0 && !running) {
			mElevatorTalon.set(ControlMode.PercentOutput, 0.1);
			timeSinceMotorCommand = 0;
		} else {
			if (controller.getBumper(Hand.kRight) && !upPressed) {
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
			
			if (timeSinceMotorCommand < 100 && Math.abs(previous - mElevatorTalon.getSelectedSensorPosition(0)) < 10 && encoderHealthy) {
				encoderHealthy = false;
				DriverStation.reportWarning("ENCODER NOT HEALTHY, ENTERING FAILSAFE MODE: " + timeSinceMotorCommand + " " + atTarget() + " " + previous + " " + mElevatorTalon.getSelectedSensorPosition(0), false);
			}

			
			if (running && !atTarget()) {
				if (encoderHealthy) {
					mElevatorTalon.set(ControlMode.MotionMagic, encoderLevels[targetLevel.ordinal()]);
					timeSinceMotorCommand += 20;

				} else {
					mElevatorTalon.set(ControlMode.PercentOutput, 0);
				}
			}
		}

		if (controller.getStartButton()) {
			override.set(true);
			LEDStrip.sendColor(LEDMode.eBlink);
		}

		if (controller.getYButton() && !yPressed) {
			tiltSolenoid.set(tiltSolenoid.get() == Value.kForward ? Value.kReverse : Value.kForward);
		}

		if (mElevatorTalon.getSensorCollection().isRevLimitSwitchClosed()) {
			mElevatorTalon.setSelectedSensorPosition(0, 0, 0);
		}
		// System.out.println("Solenoid: " + climberSolenoid.get());
		if (controller.getStickButton(Hand.kRight) && !r3Pressed) {
			if (climberSolenoid.get() == Value.kOff) {
				climberSolenoid.set(Value.kReverse);
			} else {
				climberSolenoid.set(climberSolenoid.get() == Value.kReverse ? Value.kForward : Value.kReverse);
			}
		}

		if (controller.getBackButton()
				&& deadband(-controller.getY(Hand.kRight), 0.1) != 0/* && climberSolenoid.get() == Value.kForward */) {
			double o = deadband(-controller.getY(Hand.kRight), 0.1);
			if (climberVictor.getOutputCurrent() > 50) {
				o *= 0.5;
			}
			climberVictor.set(ControlMode.PercentOutput, o);
		} else {
			climberVictor.set(ControlMode.PercentOutput, 0.0);
		}

		r3Pressed = controller.getStickButton(Hand.kRight);

		/*
		 * if (controller.getStartButton() && !startPressed) { targetLevel =
		 * ElevatorPosition.kGround; executeHead(); }
		 * 
		 * if (getDelta(elevatorLevel) > Constants.kElevatorThreshhold) { for
		 * (ElevatorPosition el : ElevatorPosition.values()) { if (getDelta(el) <
		 * Constants.kElevatorThreshhold) { elevatorLevel = el; attaralse; } } }
		 * 
		 * if (running) { mElevatorTalon.set(ControlMode.Position,
		 * encoderLevels[targetLevel.ordinal()]); }
		 * 
		 */

		upPressed = controller.getBumper(Hand.kRight);

		downPressed = controller.getBumper(Hand.kLeft);
		ltPressed = deadband(controller.getTriggerAxis(Hand.kLeft), 0.5) != 0;
		L3Pressed = controller.getStickButton(Hand.kLeft);
		nudgeUpPressed = controller.getPOV() == 0;
		yPressed = controller.getYButton();
		nudgeDownPressed = controller.getPOV() == 180;
		startPressed = controller.getStartButton();
		
		previous = mElevatorTalon.getSelectedSensorPosition(0);
	}

	public void drive() {
		if (running) {
			System.out.println("Current: " + mElevatorTalon.getSelectedSensorPosition(0) + ", Target: " + encoderLevels[targetLevel.ordinal()]);
			mElevatorTalon.set(ControlMode.Position, encoderLevels[targetLevel.ordinal()]);
			if(getDelta(targetLevel) < Constants.kElevatorThreshhold)
				running = false;
			tiltSolenoid.set(Value.kReverse);
		}
	}

	private int getDelta(ElevatorPosition position) {
		return Math.abs(currentCount - encoderLevels[position.ordinal()]);
	}

	public void initEncoder() {
		mElevatorTalon.setSelectedSensorPosition(Constants.kInitialCount, 0, 0);
	}

	public double getHeight() {
		return mElevatorTalon.getSelectedSensorPosition(0) / encoderLevels[6];
	}

	public boolean zeroLevel() {
		double time = System.currentTimeMillis();
		while (mElevatorTalon.getSelectedSensorPosition(0) > 10 && System.currentTimeMillis() - time < 5000) {
			mElevatorTalon.set(ControlMode.PercentOutput, -0.7);
		}
		return true;

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
		timeSinceMotorCommand = 0;
		if (!atTarget()) {
			running = true;
		}
	}

	public void setClimber(Value value) {
		climberSolenoid.set(value);
	}

	/**
	 * Resets target level
	 */
	private void clearEverything() {
		targetLevel = elevatorLevel;
		running = false;
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

	public void toPosition(ElevatorPosition elevatorPosition) {
		targetLevel = elevatorPosition;
		executeHead();

	}

	public boolean isFinished() {
		return atTarget();
	}
}
