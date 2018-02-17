package org.usfirst.frc.team263.robot;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Wrapper to represent a Motion Profile.
 * 
 * @version 0.0.1
 * @author Rohan Bapat
 * @since 01-29-2018
 */
public class ElevatorProfile {
	private double[] trajectories;
	private int numTrajectories, initialLevel, targetLevel;
	private boolean isGenerated;

	/**
	 * Constructor that defaults everything to zero.
	 */
	public ElevatorProfile() {
		trajectories = new double[0];
		numTrajectories = 0;
		targetLevel = 0;
		initialLevel = 0;
		isGenerated = false;
	}

	/**
	 * Getter for number of trajectories.
	 * 
	 * @return number of trajectories.
	 */
	public int getNumTrajectories() {
		return numTrajectories;
	}

	/**
	 * Sets profiles state as 'generated' or able to be sent to the TalonSRX's.
	 * buffer
	 * 
	 * @param trajectories
	 *            array of doubles [position, velocity, timeInterval] representing.
	 *            trajectories
	 * @param target
	 *            target level of elevator.
	 * @param initial
	 *            initial level of elevator.
	 */
	public void setGenerated(double[] trajectories, int target, int initial) {
		this.trajectories = trajectories;
		targetLevel = target;
		initialLevel = initial;
		numTrajectories = trajectories.length;
		isGenerated = true;
	}

	/**
	 * Getter for whether or not profile is generated.
	 * 
	 * @return whether or not the profile is generated.
	 */
	public boolean isGenerated() {
		return isGenerated;
	}

	/**
	 * Getter for targetLevel.
	 * 
	 * @return target level of profile/elevator.
	 */
	public int getTargetLevel() {
		return targetLevel;
	}

	/**
	 * Getter for initialLevel.
	 * 
	 * @return initialLevel of profile/elevator.
	 */
	public int getInitialLevel() {
		return initialLevel;
	}

	/**
	 * Parses data from trajectory array to trajectory points and pushes them to
	 * TalonSRX's top level buffer.
	 * 
	 * @param mElevatorTalon
	 *            TalonSRX to push to.
	 * @return elevator level that elevator will move to.
	 */
	public int pushBuffer(TalonSRX mElevatorTalon) {
		if (!isGenerated) {
			return -1;
		} else {
			TrajectoryPoint point = new TrajectoryPoint();
			mElevatorTalon.clearMotionProfileTrajectories();
			mElevatorTalon.configMotionProfileTrajectoryPeriod(0, 0);
			for (int x = 0; x < trajectories.length; x++) {
				point.position = trajectories[0] * Constants.kElevatorUnitsPerRotation;
				point.velocity = trajectories[1] * Constants.kElevatorUnitsPerRotation;
				point.timeDur = TrajectoryDuration.Trajectory_Duration_10ms.valueOf((int) trajectories[2]);
				point.profileSlotSelect0 = 0;

				point.headingDeg = 0;
				point.profileSlotSelect1 = 0;

				point.zeroPos = (x == 0);
				point.isLastPoint = (x == trajectories.length - 1);

				mElevatorTalon.pushMotionProfileTrajectory(point);

			}
			return targetLevel;
		}
	}

}
