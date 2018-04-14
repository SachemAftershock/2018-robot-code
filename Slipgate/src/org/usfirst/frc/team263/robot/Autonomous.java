
package org.usfirst.frc.team263.robot;

import java.util.List;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

import org.usfirst.frc.team263.robot.Enums.AutoObjective;
import org.usfirst.frc.team263.robot.Enums.Direction;
import org.usfirst.frc.team263.robot.Enums.ElevatorPosition;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Autonomous {
	private static Autonomous mInstance = new Autonomous();
	private SWDrive mDrive;
	private CubeIntake mIntake;
	private MagicElevator mElevator;
	private Queue<AutoObjective> mObjectiveQueue;
	private Queue<List<Double>> mSetpointQueue;
	private AutoObjective mObjective;
	private List<Double> mSetpoint;
	private boolean mIsObjectiveFinished;
	private double startTime;

	/**
	 * Constructor for Autonomous class.
	 */
	private Autonomous() {
		mDrive = SWDrive.getInstance();
		mIntake = CubeIntake.getInstance();
		mElevator = MagicElevator.getInstance();
		mObjectiveQueue = new LinkedList<AutoObjective>();
		mSetpointQueue = new LinkedList<List<Double>>();
		mIsObjectiveFinished = true;
	}

	/**
	 * Gets instance of singleton class Autonomous.
	 * 
	 * @return The single instance of Autonomous.
	 */
	public static Autonomous getInstance() {
		return mInstance;
	}

	/**
	 * Drives Autonomous movements in a state machine fashion.
	 * 
	 * To add objective to FIFO Queue, @see queueObjective(AutoObjective,
	 * double)
	 */
	public void drive() {
		boolean isFirst = false;
		if (!mObjectiveQueue.isEmpty() && mIsObjectiveFinished) {
			mObjective = mObjectiveQueue.poll();
			System.out.println(mObjective);
			mSetpoint = mSetpointQueue.poll();
			isFirst = true;
			startTime = System.currentTimeMillis();
			
		}

		switch (mObjective) {
		case eNothing:
			mIsObjectiveFinished = true;
			mDrive.setOpenLoop();
			mDrive.drive(0, 0);
			break;
		case eForward:
			if (isFirst) mDrive.setLinearDistance(mSetpoint.get(0));
			mDrive.drive();
			mIsObjectiveFinished = mDrive.isSetpointReached();
			break;
		case eEjectCube:
			mIntake.autonEjectCube();
			mIsObjectiveFinished = true;
			break;
		case eRotate:
			if (isFirst) {mDrive.setRotationTheta(mSetpoint.get(0)); System.out.println(mDrive.isSetpointReached());}
			mDrive.drive();
			mIsObjectiveFinished = mDrive.isSetpointReached() && !isFirst;
			break;
		case eCubeAssist:
			if (isFirst) mDrive.setCubeAssist(mSetpoint.get(0) == 0 ? Direction.eClockwise : Direction.eCounterclockwise);
			mDrive.drive(0, 0);
			mIsObjectiveFinished = mDrive.isSetpointReached();
			break;
		case eCurve:
			// TODO: Add curved reverse kinematic expressions after SWDrive
			// matures the feature.
			mIsObjectiveFinished = true;
			break;
		/*
		case eElevatorLevel:
			if (isFirst) mElevator.toPosition(ElevatorPosition.values()[mSetpoint.get(0).intValue()]);	
			mElevator.drive();
			mIsObjectiveFinished = mElevator.isFinished();
			break;
		case eDriveAndElevator:
			if (isFirst) {
				mElevator.toPosition(ElevatorPosition.values()[mSetpoint.get(0).intValue()]);
				mDrive.setLinearDistance(mSetpoint.get(1));
			}
			mElevator.drive();
			mDrive.drive();
			mIsObjectiveFinished = mDrive.isSetpointReached() && mElevator.isFinished();
		*/
		case eOpenArm:
			mIntake.autonOpenArm();
			mIsObjectiveFinished = true;
		case eIntake:
			mIntake.autonIntake();
			mIsObjectiveFinished = true;
		case eTriggerClimber:
			mElevator.setClimber(Value.kReverse);
			
		}
		if (System.currentTimeMillis() - startTime > 2500 && mObjective != AutoObjective.eCubeAssist) mIsObjectiveFinished = true;
	}
	
	/**
	 * Clears the autonomous queue for both objectives and setpoints.
	 */
	public void clearQueue() {
		mObjectiveQueue = new LinkedList<AutoObjective>();
		mSetpointQueue = new LinkedList<List<Double>>();
	}

	/**
	 * Queues an additional autonomous objective segment.
	 * 
	 * Currently supported objectives and setpoint pairs:
	 * - eForward: distance in inches.
	 * - eNothing: any valid double value.
	 * - eEjectCube: any valid double value.
	 * 
	 * @param objective
	 *            Objective to queue.
	 * @param setpoint
	 *            Setpoint for given objective.
	 */
	public void queueObjective(AutoObjective objective, List<Double> setpoint) {
		mObjectiveQueue.add(objective);
		mSetpointQueue.add(setpoint);
	}
	
	/**
	 * Queues an additional autonomous objective segment.
	 * 
	 * Currently supported objectives and setpoint pairs:
	 * - eForward: distance in inches.
	 * - eNothing: any valid double value.
	 * - eEjectCube: any valid double value.
	 * 
	 * @param objective
	 *            Objective to queue.
	 * @param setpoint
	 *            Setpoint for given objective.
	 */
	public void queueObjective(AutoObjective objective, double setpoint) {
		mObjectiveQueue.add(objective);
		mSetpointQueue.add(Arrays.asList(setpoint));
	}
}
