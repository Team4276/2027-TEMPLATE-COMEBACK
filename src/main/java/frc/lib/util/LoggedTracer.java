package frc.lib.util;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

/** Utility class for logging code execution times. */
public class LoggedTracer {
	private LoggedTracer() {
	}

	private static double startTime = -1.0;

	/** Reset the clock. */
	public static void reset() {
		startTime = Timer.getMonotonicTimestamp();
	}

	/** Save the time elapsed since the last reset or record. */
	public static void record(String epochName) {
		double now = Timer.getMonotonicTimestamp();
		Logger.recordOutput(
				"Logged Tracer/" + epochName + " Milliseconds", Units.secondsToMilliseconds(now - startTime));
		startTime = now;
	}
}
