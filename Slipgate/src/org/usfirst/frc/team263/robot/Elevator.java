package org.usfirst.frc.team263.robot;

import java.util.concurrent.ConcurrentLinkedQueue;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Notifier;
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
	private MotionProfileStatus profileStatus;
	private DigitalInput topSwitch, bottomSwitch;
	private TalonSRX mElevatorTalon;
	private boolean upPressed, downPressed, L3Pressed, RTPressed;
	private boolean profileReady, profileRunning, shouldStream;
	private Notifier bufferProcessor;
	private Thread streamerThread;
	private int cumulativeTarget;
	private int elevatorLevel;
	private int[] encoderLevels;
	private ConcurrentLinkedQueue<Integer> targetQueue;
	private ConcurrentLinkedQueue<ElevatorProfile> trajectoryQueue;

	/**
	 * Constructor for Elevator class.
	 */
	private Elevator() {
		// configure TalonSRX for closed-loop control
		mElevatorTalon = new TalonSRX(Constants.kLeftMasterDrivePort);
		mElevatorTalon.setNeutralMode(NeutralMode.Brake);
		mElevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		mElevatorTalon.setSensorPhase(true);
		mElevatorTalon.config_kP(0, Constants.kElevatorKp, 0);
		mElevatorTalon.config_kI(0, Constants.kElevatorKi, 0);
		mElevatorTalon.config_kD(0, Constants.kElevatorKd, 0);
		mElevatorTalon.config_kF(0, Constants.kElevatorKf, 0);
		profileStatus = new MotionProfileStatus();

		// Limit Switches for upper and lower bounds of elevator
		topSwitch = new DigitalInput(Constants.kElevatorTopLimitSwitch);
		bottomSwitch = new DigitalInput(Constants.kElevatorBottomLimitSwitch);

		// buffer and trajectory processor threads
		bufferProcessor = new Notifier(new BufferProcessor());
		streamerThread = new Thread();

		// Concurrent Queues to store data manipulated between threads
		targetQueue = new ConcurrentLinkedQueue<Integer>();
		trajectoryQueue = new ConcurrentLinkedQueue<ElevatorProfile>();

		// elevator and encoder levels
		cumulativeTarget = 0;
		elevatorLevel = 0;

		// TODO: replace with actual values taken from open-loop control on elevator
		encoderLevels = new int[] { 118, 254, 263, 353, 1156 };

		// booleans for button presses and profile availablility
		upPressed = false;
		downPressed = false;
		L3Pressed = false;
		RTPressed = false;
		profileReady = false;
		profileRunning = false;
		shouldStream = false;

		clearEverything();
	}

	/**
	 * Gets instance of singleton Elevator.
	 * 
	 * @return Gets the one instance of Elevator.
	 */
	public static Elevator getInstance() {
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
			mElevatorTalon.set(ControlMode.PercentOutput, deadband(controller.getY(Hand.kLeft), 0.1));
		} else if (controller.getAButton() && !upPressed && !topSwitch.get()) {
			if (deadband(controller.getTriggerAxis(Hand.kRight), 0.5) != 0) {
				cumulativeTarget = Math.min(cumulativeTarget + 1, encoderLevels.length - 1);
			} else {
				targetQueue.add(Math.min(elevatorLevel + 1, encoderLevels.length - 1));
				pingStreamer();
			}

		} else if (controller.getBButton() && !downPressed && !bottomSwitch.get()) {
			if (deadband(controller.getTriggerAxis(Hand.kRight), 0.5) != 0) {
				cumulativeTarget = Math.max(cumulativeTarget - 1, 0);
			} else {
				targetQueue.add(Math.max(elevatorLevel - 1, 0));
				pingStreamer();
			}
		} else if (controller.getBumper(Hand.kLeft) && controller.getBumper(Hand.kRight)) {
			clearEverything();
		} else if (controller.getStickButton(Hand.kLeft) && !L3Pressed) {
			executeHead();
		} else if (deadband(controller.getTriggerAxis(Hand.kRight), 0.5) == 0 && RTPressed) {
			targetQueue.add(cumulativeTarget);
		}

		talonCheck();

		upPressed = controller.getAButton();
		downPressed = controller.getBackButton();
		L3Pressed = controller.getStickButton(Hand.kLeft);
		RTPressed = (deadband(controller.getTriggerAxis(Hand.kRight), 0.5) == 0);
	}

	/**
	 * Wrapper method to send elevator to ground.
	 */
	public void toGround() {
		targetQueue.add(0);
		executeHead();
	}

	/**
	 * Wrapper method to send elevator to scale height.
	 */
	public void toScale() {
		targetQueue.add(4);
		executeHead();
	}

	/**
	 * Wrapper method to send elevator to switch height.
	 */
	public void toSwitch() {
		targetQueue.add(1);
		executeHead();
	}

	/**
	 * Wrapper method to send scale to given level.
	 * 
	 * @param level
	 *            level to send elevator to.
	 */
	public void toCustom(int level) {
		if (level < 0 || level >= encoderLevels.length)
			return;
		targetQueue.add(level);
		executeHead();
	}

	/**
	 * Checks if profileStreamer is running; starts it if it isn't.
	 */
	private void pingStreamer() {
		if (streamerThread.isAlive())
			return;

		streamerThread = new Thread(new ProfileStreamer(elevatorLevel));
		shouldStream = true;
		streamerThread.start();
	}
	
	/**
	 * @return If a motion profile is currently running.
	 */
	public boolean isFinished() {
		return !profileRunning && trajectoryQueue.size() == 0;
	}

	/**
	 * Executes the command at the head of the trajectory queue.
	 * 
	 * Not the Maximilien Robespierre style though.
	 */
	private void executeHead() {
		if (trajectoryQueue.size() > 0 && trajectoryQueue.peek().isGenerated()) {
			bufferProcessor.startPeriodic(0.005);
			elevatorLevel = trajectoryQueue.poll().pushBuffer(mElevatorTalon);
			cumulativeTarget = elevatorLevel;
			profileRunning = true;
		} else {
			// notify driver
			// new Thread(new ControllerRumble(controller, 2)).start();
		}
	}

	/**
	 * Checks on talon with the profile it retrieved. Handles buffer sizes and
	 * buffer underflows.
	 */
	private void talonCheck() {
		mElevatorTalon.getMotionProfileStatus(profileStatus);

		if (profileStatus.hasUnderrun) {
			mElevatorTalon.clearMotionProfileHasUnderrun(0);
		}

		if (profileStatus.btmBufferCnt > Constants.kMinBufferSize) {
			profileReady = true;

		}

		if (profileStatus.activePointValid && profileStatus.isLast) {
			profileRunning = false;
			profileReady = false;
			mElevatorTalon.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
		}

		if (profileReady && profileRunning) {
			mElevatorTalon.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
		}

		/*
		 * TODO: add data streamer that sends interpretable info to driver station,
		 * either with ShuffleBoard, SmartDashboard, or UDP Sockets
		 */
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

	/**
	 * Clears everything in commandQueues, terminates threads, and clears talon.
	 * buffer
	 */
	private void clearEverything() {
		shouldStream = false;
		targetQueue.clear();
		trajectoryQueue.clear();
		bufferProcessor.stop();
		mElevatorTalon.clearMotionProfileTrajectories();
		mElevatorTalon.clearMotionProfileHasUnderrun(0);
	}

	/**
	 * Runnable used in notifier to move top-level buffer data to the bottom
	 * firmware buffer of TalonSRX.
	 * 
	 * @author Rohan Bapat
	 *
	 */
	class BufferProcessor implements Runnable {

		/**
		 * Makes call to move data
		 */
		public void run() {
			mElevatorTalon.processMotionProfileBuffer();
		}
	}

	/**
	 * Runnable used to take queued commands and convert them to trapezoidal motion
	 * profile trajectory points.
	 * 
	 * @author Rohan Bapat
	 *
	 */
	class ProfileStreamer implements Runnable {
		private int prevLevel;

		/**
		 * default constructor where elevator level is defaulted to 0.
		 */
		public ProfileStreamer() {
			prevLevel = 0;
		}

		/**
		 * constructor where current elevator level is given.
		 * 
		 * @param currentLevel
		 *            current elevator level
		 */
		public ProfileStreamer(int currentLevel) {
			prevLevel = currentLevel;
		}

		/**
		 * Thread-body where command is taken from the targetQueue, converted to motion
		 * profile trajectories, and sent to trajectoryQueue.
		 */
		public void run() {
			while (targetQueue.size() > 0 && shouldStream) {
				int targetLevel = targetQueue.poll();
				int targetPos = encoderLevels[targetLevel];
				int currentPos = encoderLevels[prevLevel];
				ElevatorProfile prof = new ElevatorProfile();
				// TODO: add automatic profile generation, will work out kinematics tomorrow
				prof.setGenerated(new double[] { 0.0, 1.0, 2.0 }, targetLevel, prevLevel);
				trajectoryQueue.add(prof);
				prevLevel = targetLevel;
			}
		}
	}

}
