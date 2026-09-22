package frc.lib.bases;

import org.littletonrobotics.junction.Logger;

import org.wpilib.units.BaseUnits;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;
import org.wpilib.command2.*;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.DelayedBoolean;
import frc.lib.util.Util;

/**
 * Base subsystem for any subsystem that uses motors and requires precise
 * position control.
 */
public class ServoMotorSubsystem<IO extends MotorIO> extends MotorSubsystem<IO> {
	protected final Angle epsilonThreshold;

	// Variables for handling homing logic
	protected boolean isHomingSubsystem;
	private ServoHomingConfig homingConfig;
	private boolean mHoming = false;
	private boolean mNeedsToHome = true;
	private DelayedBoolean mHomingDelay;

	/**
	 * Creates a *non-homing* ServoMotorSubsystem with a MotorIO, name for
	 * telemetry, and threshold for differences in measurement.
	 *
	 * @param io               MotorIO for the subsystem.
	 * @param name             Name for telemetry.
	 * @param epsilonThreshold Acceptable error range for position.
	 */
	public ServoMotorSubsystem(IO io, String name, Angle epsilonThreshold, boolean tuningMode) {
		super(io, name, tuningMode);
		this.isHomingSubsystem = false;
		this.epsilonThreshold = epsilonThreshold;
	}

	public ServoMotorSubsystem(IO io, String name, Angle epsilonThreshold) {
		this(io, name, epsilonThreshold, false);
	}

	/**
	 * Creates a *homing* ServoMotorSubsystem with a MotorIO, name for telemetry,
	 * and threshold for differences in measurement.
	 *
	 * @param io               MotorIO for the subsystem.
	 * @param name             Name for telemetry.
	 * @param epsilonThreshold Acceptable error range for position.
	 * @param config           Homing configuration.
	 */
	public ServoMotorSubsystem(
			IO io, String name, Angle epsilonThreshold, boolean tuningMode, ServoHomingConfig config) {
		this(io, name, epsilonThreshold, tuningMode);
		this.isHomingSubsystem = true;
		homingConfig = config;
		mHomingDelay = new DelayedBoolean(Timer.getMonotonicTimestamp(), homingConfig.kHomingTimeout.in(Units.Seconds));
	}

	public ServoMotorSubsystem(IO io, String name, Angle epsilonThreshold, ServoHomingConfig config) {
		this(io, name, epsilonThreshold, false, config);
	}

	@Override
	public void periodic() {
		super.periodic();
		if (isHomingSubsystem) {
			if (mNeedsToHome && setpointNearHome() && nearHomingLocation()) {
				mHoming = true;
				useSoftLimits(false);
				mHomingDelay = new DelayedBoolean(Timer.getMonotonicTimestamp(),
						homingConfig.kHomingTimeout.in(Units.Seconds));
			}
			if (mHoming) {
				io.applySetpoint(Setpoint.withVoltageSetpoint(homingConfig.kHomingVoltage));
				if (mHomingDelay.update(
						Timer.getMonotonicTimestamp(),
						Math.abs(getVelocity().baseUnitMagnitude()) < homingConfig.kSetHomedVelocity.baseUnitMagnitude()
								&& RobotBase.isEnabled())) {
					setCurrentPosition(homingConfig.kHomePosition);
					applySetpoint(Setpoint.withMotionMagicSetpoint(homingConfig.kHomePosition));
					useSoftLimits(true);
					mNeedsToHome = false;
				}
			}
		}

		Logger.recordOutput(name + "/Homing", mHoming);
		Logger.recordOutput(name + "/Needs to Home", mNeedsToHome);
	}

	/**
	 * Determines whether the currently set setpoint is near subsystem's home
	 * position.
	 *
	 * @return True if setpoint is near the home position, false if not.
	 */
	public boolean setpointNearHome() {
		return getSetpoint().mode.isPositionControl()
				&& Util.epsilonEquals(
						getSetpointDoubleInUnits(),
						homingConfig.kHomePosition.in(io.unitType),
						epsilonThreshold.in(io.unitType));
	}

	/**
	 * Determines whether the subsystem is near it's homing position.
	 *
	 * @return True if currently near home position, false if not.
	 */
	public boolean nearHomingLocation() {
		return nearPosition(homingConfig.kHomePosition);
	}

	/**
	 * Determines whether the subsystem is near it's position setpoint.
	 *
	 * @return True if currently near setpoint, false if not. Returns false if not
	 *         in position control.
	 */
	public boolean nearPositionSetpoint() {
		return (getSetpoint().mode.isPositionControl()) && nearPosition(getPosition());
	}

	/**
	 * Gets the current setpoint value of the MotorIO using units of the MotorIO.
	 *
	 * @return Setpoint in mechanism units.
	 */
	public double getSetpointDoubleInUnits() {
		return io.getSetpointDoubleInUnits();
	}

	/**
	 * Determines whether the subsystem is near a given position.
	 *
	 * @param mechanismPosition Position to compare to.
	 * @return True if near provided position, false if not.
	 */
	public boolean nearPosition(Angle mechanismPosition) {
		return Util.epsilonEquals(
				getPosition().in(BaseUnits.AngleUnit),
				mechanismPosition.in(BaseUnits.AngleUnit),
				epsilonThreshold.in(BaseUnits.AngleUnit));
	}

	/**
	 * Determines whether the subsystem is near a given position.
	 *
	 * @param mechanismPosition Position to compare to.
	 * @param epsilon           Acceptable error range for position
	 * @return True if near provided position, false if not.
	 */
	public boolean nearPosition(Angle mechanismPosition, Angle epsilon) {
		return Util.epsilonEquals(
				getPosition().in(BaseUnits.AngleUnit),
				mechanismPosition.in(BaseUnits.AngleUnit),
				epsilon.in(BaseUnits.AngleUnit));
	}

	@Override
	public void applySetpoint(Setpoint setpoint) {
		super.applySetpoint(setpoint);
		if (isHomingSubsystem) {
			if (mHoming) {
				mHoming = false;
				useSoftLimits(true);
			}
			if (!setpointNearHome()) {
				mNeedsToHome = true;
			}
		}
	}

	/**
	 * Creates a Command that waits until the mechanism is near a given position.
	 *
	 * @param mechanismPosition Position to evaluate proximity to.
	 * @return A wait command.
	 */
	public Command waitForPositionCommand(Angle mechanismPosition) {
		return Commands.waitUntil(() -> {
			return nearPosition(mechanismPosition);
		});
	}

	/**
	 * Creates a Command that goes to a setpoint and then waits until the mechanism
	 * is the setpoint's position.
	 *
	 * @param mechanismPosition Position to evaluate proximity to.
	 * @return A new Command to apply setpoint and wait.
	 */
	public Command setpointCommandWithWait(Setpoint setpoint) {
		return waitForPositionCommand(BaseUnits.AngleUnit.of(setpoint.baseUnits))
				.deadlineFor(setpointCommand(setpoint));
	}

	/**
	 * Enables or disables soft limits.
	 *
	 * @param enable True to enable, soft to disable.
	 */
	public void useSoftLimits(boolean enable) {
		io.useSoftLimits(enable);
	}

	/**
	 * Sets neutral mode to brake or coast.
	 *
	 * @param wantsBrake True for brake, false for coast.
	 */
	public void setNeutralBrake(boolean wantsBrake) {
		io.setNeutralBrake(wantsBrake);
	}

	/**
	 * Set's the mechanism's current location as a given position.
	 *
	 * @param mechanismPosition the mechanism's position to set location as.
	 */
	public void setCurrentPosition(Angle position) {
		io.setCurrentPosition(position);
	}

	/**
	 * Configuration to make a homing ServoMotorSubsystem
	 */
	public static class ServoHomingConfig {
		public Angle kHomePosition;
		public Voltage kHomingVoltage;
		public Time kHomingTimeout;
		public AngularVelocity kSetHomedVelocity;
	}
}
