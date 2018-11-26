package org.usfirst.frc.team263.robot.Enums;

/**
 * Enum to control current state of LED Strip
 * 
 * @author Rohan Bapat
 * @author Dan Waxman
 *
 */
public enum LEDMode {
	eRed, eBlue, ePink, eTeal, eOff, eRainbow, eBlink,
	eOverrideToggle, eTeleop, eIngest, eExpel, eHighGear, eLowGear, eBrownout, eProfileReady;
	// ^^ took off eGreen bc Vision System light rings are same color
}