package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Creates a Thread to vibrate joystick
 * 
 * @author Dan Waxman
 * @since 02-14-2016
 * @version 1.1
 */
public class ControllerRumble extends Thread {
	public static boolean exists;
	private double duration;
	private XboxController[] joysticks;
	private int amount;

	/**
	 * @param stick
	 *            XboxController to rumble
	 * @param occurence
	 *            Amount of times to rumble
	 */
	public ControllerRumble(XboxController stick, int occurence) {
		this(new XboxController[] { stick }, occurence, 0.25);
	}

	/**
	 * @param stick
	 *            XboxController to rumble
	 * @param occurence
	 *            Amount of times to rumble
	 * @param length
	 *            Duration of each rumble
	 */
	public ControllerRumble(XboxController stick, int occurence, double length) {
		this(new XboxController[] { stick }, occurence, length);
	}

	/**
	 * @param sticks
	 *            XboxControllers to rumble
	 * @param occurence
	 *            Amount of times to rumble
	 */
	public ControllerRumble(XboxController[] sticks, int occurence) {
		this(sticks, occurence, 0.25);
	}

	/**
	 * @param sticks
	 *            XboxControllers to rumble
	 * @param occurence
	 *            Amount of times to rumble
	 * @param length
	 *            Duration of rumble
	 */
	public ControllerRumble(XboxController[] sticks, int occurence, double length) {
		duration = length;
		joysticks = sticks;
		amount = occurence;
		
		exists = true;
	}

	/**
	 * Starts thread where XboxController will rumble
	 */
	@Override
	public void run() {
		for (int i = 0; i < amount; i++) {
			for (XboxController drivepad : joysticks) {
				drivepad.setRumble(RumbleType.kLeftRumble, 1f);
				drivepad.setRumble(RumbleType.kRightRumble, 1f);
			}

			Timer.delay(duration);

			for (XboxController drivepad : joysticks) {
				drivepad.setRumble(RumbleType.kLeftRumble, 0f);
				drivepad.setRumble(RumbleType.kRightRumble, 0f);
			}

			Timer.delay(duration);
		}

		joysticks = null;
		exists = false;
	}
}
