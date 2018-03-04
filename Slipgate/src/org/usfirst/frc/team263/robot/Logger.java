package org.usfirst.frc.team263.robot;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class for Logging robot operations.
 * 
 * @author Dan Waxman
 */
public class Logger {
	private BufferedWriter mWriter;
	private boolean properInstance = false;

	/**
	 * Creates a new Logger object with a new file.
	 * 
	 * @throws IOException
	 */
	public Logger() {
		try {
			SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd_HH.mm.ss");
			String fname = "/U/ROBORIO_LOG_" + dateFormat.format(new Date()) + ".log";
			FileWriter fstream = new FileWriter(fname);
			mWriter = new BufferedWriter(fstream);
			properInstance = true;
			System.out.println("Created Logger");
		} catch (IOException e) {
			DriverStation.reportError("Problems instantiating logger.", false);
			properInstance = false;
		}
	}

	/**
	 * Writes a string to log file with optional syncing.
	 * 
	 * @param s
	 *            String to append to log file.
	 * @param forcedSync
	 *            If true, file will be synced. This increases overhead but will
	 *            prevent loss of information in the case of accidental exit.
	 * @return 0 if successful, -1 if error occurred.
	 */
	public int write(String s, boolean forcedSync) {
		if (!properInstance) {
			return -1;
		} 
		try {
			mWriter.write(System.nanoTime() + "\t : \t" + s + '\n');
			
			if (forcedSync) {
				mWriter.flush();
			}
		} catch (IOException e) {
			DriverStation.reportError("Logging Error", false);
			return -1;
		}

		return 0;
	}

	/**
	 * Closes log file.
	 * 
	 * @return 0 if successful, -1 if error occurred.
	 */
	public int close() {
		if (!properInstance) {
			return -1;
		}
		try {
			mWriter.flush();
			mWriter.close();
		} catch (IOException e) {
			DriverStation.reportError("Logging Error", false);
			return -1;
		}

		return 0;
	}

	/**
	 * Forces log file to sync. This adds a lot of overhead and should be
	 * avoided, but is useful if robot might unexpectedly lose power.
	 * 
	 * @return 0 if successful, -1 otherwise.
	 */
	public int forceSync() {
		if (!properInstance) {
			return -1;
		}
		try {
			mWriter.flush();
		} catch (IOException e) {
			DriverStation.reportError("LoggingError", false);
			return -1;
		}

		return 0;
	}
}
