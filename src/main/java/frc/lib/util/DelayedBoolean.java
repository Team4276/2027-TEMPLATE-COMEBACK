package frc.lib.util;

/**
 * An iterative boolean latch that delays the transition from false to true.
 */
public class DelayedBoolean {
	private boolean mLastValue;
	private double mTransitionTimestamp;
	private final double mDelay;

	public DelayedBoolean(double timestamp, double delay) {
		mTransitionTimestamp = timestamp;
		mLastValue = false;
		mDelay = delay;
	}

	public boolean update(double timestamp, boolean value) {
		// Timestamp and delay use the same units (callers use seconds). False clears immediately;
		// every rising edge starts a new dwell, so intermittent true samples cannot accumulate.
		boolean result = false;

		if (value && !mLastValue) {
			mTransitionTimestamp = timestamp;
		}

		// If we are still true and we have transitioned.
		if (value && (timestamp - mTransitionTimestamp > mDelay)) {
			result = true;
		}

		mLastValue = value;
		return result;
	}
}
