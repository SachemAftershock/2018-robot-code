package org.usfirst.frc.team263.robot;

import java.util.List;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

import org.usfirst.frc.team263.robot.Enums.AutoObjective;
import org.usfirst.frc.team263.robot.Enums.Direction;

public class Autonomous {
	private static Autonomous mInstance = new Autonomous();
	private SWDrive mDrive;
	private CubeIntake mIntake;
	private Queue<AutoObjective> mObjectiveQueue;
	private Queue<List<Double>> mSetpointQueue;
	private AutoObjective mObjective;
	private List<Double> mSetpoint;
	private boolean mIsObjectiveFinished;

	/**
	 * Constructor for Autonomous class.
	 */
	private Autonomous() {
		mDrive = SWDrive.getInstance();
		mIntake = CubeIntake.getInstance();
		mObjectiveQueue = new LinkedList<AutoObjective>();
		mSetpointQueue = new LinkedList<List<Double>>();
		mIsObjectiveFinished = true;
		mDrive.zeroGyro();
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
			mSetpoint = mSetpointQueue.poll();
			isFirst = true;
		}

		switch (mObjective) {
		case eNothing:
			mIsObjectiveFinished = true;
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
			if (isFirst) mDrive.setRotationTheta(mSetpoint.get(0));
			mDrive.drive();
			mIsObjectiveFinished = mDrive.isSetpointReached();
			break;
		case eCubeAssist:
			if (isFirst) mDrive.setCubeAssist(mSetpoint.get(0) == 0 ? Direction.eClockwise : Direction.eCounterclockwise);
			mDrive.drive();
			mIsObjectiveFinished = mDrive.isSetpointReached();
			break;
		case eCurve:
			// TODO: Add curved reverse kinematic expressions after SWDrive
			// matures the feature.
			mIsObjectiveFinished = true;
			break;
		}
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
