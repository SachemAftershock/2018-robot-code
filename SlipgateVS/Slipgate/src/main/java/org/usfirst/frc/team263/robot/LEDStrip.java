package org.usfirst.frc.team263.robot;

import org.usfirst.frc.team263.robot.Enums.LEDMode;

import edu.wpi.first.wpilibj.I2C;

/**
 * Class to control LED strip on shirt shooter robot over Rio-Arduino
 * communication
 * 
 * @author Rohan Bapat
 * @author Dan Waxman
 * @version 1.3
 * @since 01/29/17
 */
public class LEDStrip {
	private static I2C i2c = new I2C(I2C.Port.kOnboard, Constants.kArduinoI2CAddress);
	private static byte[] colorModes = { 'r', 'g', 'b', 'p', 't', 'n', 'a', 'o', '0', '1', '2', '3', '4', '5', '6', '7'};
	public static LEDMode currentMode = LEDMode.eOff;

	/**
	 * Send current desired LED Strip mode over I2C to Arduino
	 * 
	 * @param color
	 *            LEDMode enum element to set LEDStrip
	 */
	public static void sendColor(LEDMode color) {
		// Test to make sure color is different than current mode to avoid
		// redundancy
		if (!color.equals(currentMode)) {
			i2c.writeBulk(new byte[] { colorModes[color.ordinal()] });
			currentMode = color;
		}
	}
}