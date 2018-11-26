package org.usfirst.frc.team263.robot;

/**
 * Class for kinematic calculations of differential drive system.
 * 
 * @author Dan Waxman
 */
public class Kinematics {
	/**
	 * Gets the signed distance to ICC based upon drive parameters.
	 * 
	 * @param l
	 *            Length from left wheel to right wheel.
	 * @param vl
	 *            Velocity of left wheel.
	 * @param vr
	 *            Velocity of right wheel.
	 * @return Signed distance to ICC.
	 */
	public static double getDistanceToICC(double l, double vl, double vr) {
		return (l / 2) * (vl + vr) / (vr - vl);
	}

	/**
	 * Gets the rate of rotation based upon drive parameters.
	 * 
	 * @param l
	 *            Length from left wheel to right wheel.
	 * @param vl
	 *            Velocity of left wheel.
	 * @param vr
	 *            Velocity of right wheel.
	 * @return Rate of rotation of drivetrain.
	 */
	public static double getOmega(double l, double vl, double vr) {
		return (vr - vl) / l;
	}

	/**
	 * Gets the ratio of velocities to achieve a given ICC.
	 * 
	 * @param r
	 *            Signed distance to ICC.
	 * @param l
	 *            Length from left wheel to right wheel.
	 * @return V_l / V_r
	 */
	public static double getVelocityRatio(double r, double l) {
		return (2 * r / l - 1) / (2 * r / l + 1);
	}
}
