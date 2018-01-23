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

	/**
	 * Creates a new Logger object with a new file.
	 * 
	 * @throws IOException
	 */
	public Logger() throws IOException {
		SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd_HH.mm.ss");
		String fname = "/U/ROBORIO_LOG_" + dateFormat.format(new Date()) + ".log";
		FileWriter fstream = new FileWriter(fname);
		mWriter = new BufferedWriter(fstream);
		System.out.println("Created Logger");
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
		try {
			mWriter.flush();
		} catch (IOException e) {
			DriverStation.reportError("LoggingError", false);
			return -1;
		}

		return 0;
	}
}
