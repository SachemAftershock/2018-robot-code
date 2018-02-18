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
	private int numTrajectories, index;
	private boolean isGenerated;

	/**
	 * Constructor that defaults everything to zero.
	 */
	public ElevatorProfile() {
		reset();
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
	public void setGenerated(double[] trajectories) {
		this.trajectories = trajectories;
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
	 * Parses data from trajectory array to trajectory points and pushes them to
	 * TalonSRX's top level buffer.
	 * 
	 * @param mElevatorTalon
	 *            TalonSRX to push to.
	 * @param amount
	 * 			  amount of points to stream to talon.
	 * 
	 * @return elevator level that elevator will move to.
	 */
	public void pushBuffer(TalonSRX mElevatorTalon, int amount) {
		if (!isGenerated) {
			return;
		} else {
			TrajectoryPoint point = new TrajectoryPoint();
			mElevatorTalon.clearMotionProfileTrajectories();
			mElevatorTalon.configMotionProfileTrajectoryPeriod(0, 0);
			if(index + amount > trajectories.length)
				amount = trajectories.length - index;
				
			for (int x = index; x < index + amount; x++) {
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
		}
	}
	
	public void reset() {
		trajectories = new double[0];
		numTrajectories = 0;
		index = 0;
		isGenerated = false;
	}

}
