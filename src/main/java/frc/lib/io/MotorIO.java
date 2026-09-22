package frc.lib.io;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.BaseUnits;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Units;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.*;
import frc.lib.util.Util;

import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.AutoLog;

/**
 * Abstract class used to control a main motor and any number of followers for a
 * mechanism.
 */
public abstract class MotorIO {
	
	@AutoLog
	public static class MotorIOInputs {
		public boolean enabled = true;
		public Mode setPointType = Mode.IDLE;
		public double setPointValueAsDouble = 0.0;

		public double[] velocity = { 0.0 };
		public double[] position = { 0.0 };
		public double[] statorCurrent = { 0.0 };
		public double[] supplyCurrent = { 0.0 };
		public double[] motorVoltage = { 0.0 };
		public double pidVoltage = 0.0;
		public double[] motorTemperature = { 0.0 };
		public double[] acceleration = { 0.0 };

		// public AngularVelocity[] velocity = {
		// BaseUnits.AngleUnit.of(0.0).per(Units.Second) };
		// public Angle[] position = { BaseUnits.AngleUnit.of(0.0) };
		// public Current[] statorCurrent = { BaseUnits.CurrentUnit.of(0.0) };
		// public Current[] supplyCurrent = { BaseUnits.CurrentUnit.of(0.0) };
		// public Voltage[] motorVoltage = { BaseUnits.VoltageUnit.of(0.0) };
		// public Voltage pidVoltage = BaseUnits.VoltageUnit.of(0.0);
		// public Temperature[] motorTemperature = { BaseUnits.TemperatureUnit.of(0.0)
		// };
		// public AngularAcceleration[] acceleration = {
		// BaseUnits.AngleUnit.of(0.0).per(Units.Seconds).per(Units.Seconds) };

		public boolean configFailed = false;
	}

	public final AngleUnit unitType;
	public final TimeUnit time;
	public final MotorIOInputsAutoLogged inputs;
	public final int numFollowers;
	private Setpoint setpoint = Setpoint.withNeutralSetpoint();
	private boolean enabled = true;

	/**
	 * Updates MotorIO's inputs with values from the motor.
	 */
	public abstract void updateInputs();

	/**
	 * Set's the mechanism's current location as a given position.
	 *
	 * @param mechanismPosition the mechanism's position to set location as.
	 */
	public void setCurrentPosition(Angle mechanismPosition) {
	}

	/**
	 * Set's the mechanism's current location as zero.
	 */
	public void zeroSensors() {
	}

	/**
	 * Sets the motor to brake or coast.
	 *
	 * @param wantsBrake Whether to brake or coast. True is brake, false is coast.
	 */
	public void setNeutralBrake(boolean wantsBrake) {
	}

	public TalonFXConfiguration getMotorIOConfig() {
		return new TalonFXConfiguration();
	}

	/**
	 * Gets whether or not the config is applied successfully to the motor
	 */
	public boolean getConfigFailed() {
		return false;
	}

	/**
	 * Sets whether to enable or disable soft limits.
	 *
	 * @param enable Whether to enable or disable soft limits. True is enable, false
	 *               is disable.
	 */
	public void useSoftLimits(boolean enable) {
	}

	/**
	 * Sets the motor to be idle. Should not be called directly, only applied
	 * through Setpoints.
	 */
	protected void setNeutralSetpoint() {
	}

	/**
	 * Sets the motor to be coasting. Should not be called directly, only applied
	 * through Setpoints.
	 */
	protected void setCoastSetpoint() {
	}

	/**
	 * Sets the motor to run at at given voltage. Should not be called directly,
	 * only applied through Setpoints.
	 *
	 * @param voltage Voltage to run at.
	 */
	protected void setVoltageSetpoint(Voltage voltage) {
	}

	/**
	 * Sets the motor to use motion magic control to go to a given position. Should
	 * not be called directly, only applied through Setpoints.
	 *
	 * @param mechanismPosition Mechanism position to go to.
	 * @param slot              The PID slot to assign
	 */
	protected void setMotionMagicSetpoint(Angle mechanismPosition, int slot) {
	}

	/**
	 * Sets the motor to use motion magic control to go to a given position. Should
	 * not be called directly, only applied through Setpoints.
	 *
	 * @param mechanismPosition Mechanism position to go to.
	 */
	protected void setMotionMagicSetpoint(Angle mechanismPosition) {
	}

	/**
	 * Sets the motor to go to a given velocity. Should not be called directly, only
	 * applied through Setpoints.
	 *
	 * @param mechanismVelocity Mechanism velocity to go to.
	 * @param slot              The PID slot to assign
	 */
	protected void setVelocitySetpoint(AngularVelocity mechanismVelocity, int slot) {
	}

	/**
	 * Sets the motor to go to a given velocity. Should not be called directly, only
	 * applied through Setpoints. DEFAULTS TO SLOT 1
	 *
	 * @param mechanismVelocity Mechanism velocity to go to.
	 */
	protected void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
	}

	/**
	 * Sets the motor to run at a percentage of it's max voltage. Should not be
	 * called directly, only applied through Setpoints.
	 *
	 * @param percent Percentage of max voltage to run at.
	 */
	protected void setDutyCycleSetpoint(Dimensionless percent) {
	}

	/**
	 * Sets the motor to use PID control to go to a given position. Should not be
	 * called directly, only applied through Setpoints.
	 *
	 * @param mechanismPosition Mechanism position to go to.
	 * @param slot              The PID slot to assign
	 */
	protected void setPositionSetpoint(Angle mechanismPosition, int slot) {
	}

	/**
	 * Sets the motor to use PID control to go to a given position. Should not be
	 * called directly, only applied through Setpoints.
	 *
	 * @param mechanismPosition Mechanism position to go to.
	 */
	protected void setPositionSetpoint(Angle mechanismPosition) {
	}

	public void setMainConfig(TalonFXConfiguration configuration) {
	}

	public void changeMainConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
	}

	public void changeFollowerConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
	}

	/**
	 * Applies a Setpoint to the MotorIO.
	 *
	 * @param setpointToApply
	 */
	public final void applySetpoint(Setpoint setpointToApply) {
		setpoint = setpointToApply;
		if (enabled) {
			setpointToApply.apply(this);
		}
	}

	public void disabledPeriodic() {
	}

	/**
	 * Enables this MotorIO. Immediatly applies the last set Setpoint including
	 * Setpoints set when disabled. MotorIO is enabled by default.
	 */
	public final void enable() {
		enabled = true;
		setpoint.apply(this);
	}

	/**
	 * Disabled this MotorIO. Setpoints can still be set when disabled but will not
	 * be applied until re-enabled.
	 */
	public final void disable() {
		enabled = false;
		Setpoint.withNeutralSetpoint().apply(this);
	}

	/**
	 * Gets whether this MotorIO is enabled.
	 *
	 * @return True if enabled, false if disabled.
	 */
	public boolean getEnabled() {
		return enabled;
	}

	/**
	 * Constructs a MotorIO with no follower motors.
	 *
	 * @param unit Units to measure.
	 * @param time Time units to measure.
	 */
	protected MotorIO(AngleUnit unit, TimeUnit time) {
		this(unit, time, 0);
	}

	/**
	 * Constructs a MotorIO with a given number of follower motors.
	 *
	 * @param unit         Units to measure in.
	 * @param time         Time units to measure.
	 * @param numFollowers The number of follower motors.
	 */
	protected MotorIO(AngleUnit unit, TimeUnit time, int numFollowers) {
		this.unitType = unit;
		this.time = time;
		inputs = new MotorIOInputsAutoLogged();
		this.numFollowers = numFollowers;
	}

	/**
	 * Gets the last read velocity of the main motor.
	 *
	 * @return Velocity of mechanism.
	 */
	public AngularVelocity getVelocity() {
		return BaseUnits.AngleUnit.of(inputs.velocity[0]).per(Units.Second);
	}

	/**
	 * Gets the last read position of the main motor.
	 *
	 * @return Position of mechanism.
	 */
	public Angle getPosition() {
		return BaseUnits.AngleUnit.of(inputs.position[0]);
	}

	/**
	 * Gets the last read stator current of the main motor.
	 *
	 * @return Stator current.
	 */
	public Current getStatorCurrent() {
		return BaseUnits.CurrentUnit.of(inputs.statorCurrent[0]);
	}

	/**
	 * Gets the last read supply current of the main motor.
	 *
	 * @return Supply current.
	 */
	public Current getSupplyCurrent() {
		return BaseUnits.CurrentUnit.of(inputs.supplyCurrent[0]);
	}

	/**
	 * Gets the last read output voltage of the main motor.
	 *
	 * @return Output voltage.
	 */
	public Voltage getMotorVoltage() {
		return BaseUnits.VoltageUnit.of(inputs.motorVoltage[0]);
	}

	/**
	 * Gets the last applied setpoint of the MotorIO.
	 *
	 * @return Last applied Setpoint.
	 */
	public Setpoint getSetpoint() {
		return setpoint;
	}

	/**
	 * Gets the current setpoint value of the MotorIO using units of the MotorIO.
	 *
	 * @return Setpoint in mechanism units.
	 */
	public double getSetpointDoubleInUnits() {
		Setpoint currentSetpoint = getSetpoint();
		switch (currentSetpoint.mode) {
			case POSITIONPID:
			case MOTIONMAGIC:
				AngleUnit positionUnit = unitType;
				return positionUnit.ofBaseUnits(currentSetpoint.baseUnits).in(positionUnit);
			case VELOCITY:
				AngularVelocityUnit velocityUnit = unitType.per(time);
				return velocityUnit.ofBaseUnits(currentSetpoint.baseUnits).in(velocityUnit);
			case VOLTAGE:
				VoltageUnit voltageUnit = Units.Volts;
				return voltageUnit.ofBaseUnits(currentSetpoint.baseUnits).in(voltageUnit);
			case DUTY_CYCLE:
				DimensionlessUnit percentUnit = Units.Percent;
				return percentUnit.ofBaseUnits(currentSetpoint.baseUnits).in(percentUnit);
			case IDLE:
			default:
				return currentSetpoint.baseUnits;
		}
	}

	/**
	 * Enum to represent different control modes for a MotorIO.
	 */
	public enum Mode {
		IDLE,
		VOLTAGE,
		MOTIONMAGIC,
		VELOCITY,
		DUTY_CYCLE,
		POSITIONPID;

		/**
		 * Gets whether the control mode is based on position. Motion Magic and Position
		 * PID control count as position.
		 *
		 * @return True if in position control, false if not.
		 */
		public boolean isPositionControl() {
			return switch (this) {
				case MOTIONMAGIC, POSITIONPID -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is based on velocity.
		 *
		 * @return True if in velocity control, false if not.
		 */
		public boolean isVelocityControl() {
			return switch (this) {
				case VELOCITY -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is neutral. Only Idle counts as neutral
		 *
		 * @return True if in velocity control, false if not.
		 */
		public boolean isNeutralControl() {
			return switch (this) {
				case IDLE -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is based on voltage. Voltage and Duty Cycle
		 * control count as voltage.
		 *
		 * @return True if in voltage control, false if not.
		 */
		public boolean isVoltageControl() {
			return switch (this) {
				case VOLTAGE, DUTY_CYCLE -> true;
				default -> false;
			};
		}
	}

	/**
	 * Setpoint for a MotorIO.
	 */
	public static class Setpoint {

		public static final Setpoint COAST = new Setpoint(
				io -> {
					io.setCoastSetpoint();
					return io;
				},
				Mode.IDLE,
				0.0);

		public static final Setpoint NEUTRAL = new Setpoint(
				io -> {
					io.setNeutralSetpoint();
					return io;
				},
				Mode.IDLE,
				0);

		private final UnaryOperator<MotorIO> applier;
		public final Mode mode;
		public final double baseUnits;

		/**
		 * Creates a setpoint with a given applier, control mode, and base units
		 * equivalent.
		 *
		 * @param applier   What to apply to MotorIO when the setpoint is set.
		 * @param mode      Control mode to register for this setpoint.
		 * @param baseUnits Setpoint's target in it's base form of units as a double.
		 */
		private Setpoint(UnaryOperator<MotorIO> applier, Mode mode, double baseUnits) {
			this.applier = applier;
			this.mode = mode;
			this.baseUnits = baseUnits;
		}

		/**
		 * Creates a setpoint with a completely custom applier, control mode, and base
		 * units.
		 *
		 * @param applier   What to apply to MotorIO when the setpoint is set.
		 * @param mode      Control mode to register for this setpoint.
		 * @param baseUnits Setpoint's target in it's base form of units as a double.
		 */
		public static Setpoint withCustomSetpoint(UnaryOperator<MotorIO> applier, Mode mode, double baseUnits) {
			return new Setpoint(applier, mode, baseUnits);
		}

		/**
		 * Creates a setpoint to use motion magic control to go to a position.
		 *
		 * @param motionMagicSetpoint Posiiton to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withMotionMagicSetpoint(Angle motionMagicSetpoint) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setMotionMagicSetpoint(motionMagicSetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.MOTIONMAGIC, motionMagicSetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a motion magic setpoint to go to a position with current limits.
		 *
		 * @param motionMagicSetpoint Velocity to go to in mechanism units.
		 * @param maxStator           Maximum stator current.
		 * @param maxSupply           Maximum supply current.
		 * @return A new Setpoint.
		 */
		public static Setpoint withMotionMagicSetpointAndCurrentLimit(
				Angle motionMagicSetpoint, Current maxStator, Current maxSupply) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				TalonFXConfiguration current = io.getMotorIOConfig();
				if (!Util.epsilonEquals(maxStator.in(Amps), current.CurrentLimits.StatorCurrentLimit)) {
					io.changeMainConfig(config -> {
						config.CurrentLimits.StatorCurrentLimit = maxStator.in(Amps);
						config.CurrentLimits.SupplyCurrentLimit = maxSupply.in(Amps);
						return config;
					});
				}
				io.setMotionMagicSetpoint(motionMagicSetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.MOTIONMAGIC, motionMagicSetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to use PID control to go to a position.
		 *
		 * @param positionSetpoint Posiiton to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withPositionSetpoint(Angle positionSetpoint) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setPositionSetpoint(positionSetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.POSITIONPID, positionSetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to go to a velocity.
		 *
		 * @param velocitySetpoint Velocity to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVelocitySetpoint(AngularVelocity velocitySetpoint) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setVelocitySetpoint(velocitySetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.VELOCITY, velocitySetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to run at a voltage.
		 *
		 * @param voltage Voltage to run at.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVoltageSetpoint(Voltage voltage) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setVoltageSetpoint(voltage);
				return io;
			};
			return new Setpoint(applier, Mode.VOLTAGE, voltage.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to use motion magic control to go to a position.
		 *
		 * @param motionMagicSetpoint Posiiton to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withMotionMagicSetpoint(Angle motionMagicSetpoint, int slot) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setMotionMagicSetpoint(motionMagicSetpoint, slot);
				return io;
			};
			return new Setpoint(applier, Mode.MOTIONMAGIC, motionMagicSetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to use PID control to go to a position.
		 *
		 * @param positionSetpoint Posiiton to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withPositionSetpoint(Angle positionSetpoint, int slot) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setPositionSetpoint(positionSetpoint, slot);
				return io;
			};
			return new Setpoint(applier, Mode.POSITIONPID, positionSetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to go to a velocity.
		 *
		 * @param velocitySetpoint Velocity to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVelocitySetpoint(AngularVelocity velocitySetpoint, int slot) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setVelocitySetpoint(velocitySetpoint, slot);
				return io;
			};
			return new Setpoint(applier, Mode.VELOCITY, velocitySetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to go to a velocity with current limits.
		 *
		 * @param velocitySetpoint Velocity to go to in mechanism units.
		 * @param maxStator        Maximum stator current.
		 * @param maxSupply        Maximum supply current.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVelocitySetpointAndCurrentLimit(
				AngularVelocity velocitySetpoint, Current maxStator, Current maxSupply) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				TalonFXConfiguration current = io.getMotorIOConfig();
				if (!Util.epsilonEquals(maxStator.in(Amps), current.CurrentLimits.StatorCurrentLimit)) {
					io.changeMainConfig(config -> {
						config.CurrentLimits.StatorCurrentLimit = maxStator.in(Amps);
						config.CurrentLimits.SupplyCurrentLimit = maxSupply.in(Amps);
						return config;
					});
				}
				io.setVelocitySetpoint(velocitySetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.VELOCITY, velocitySetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a velocity setpoint to go to a velocity with current limits.
		 *
		 * @param velocitySetpoint Velocity to go to in mechanism units.
		 * @param maxStator        Maximum stator current.
		 * @param maxSupply        Maximum supply current.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVelocitySetpointAndCurrentLimit(
				AngularVelocity velocitySetpoint, int slot, Current maxStator, Current maxSupply) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				TalonFXConfiguration current = io.getMotorIOConfig();
				if (!Util.epsilonEquals(maxStator.in(Amps), current.CurrentLimits.StatorCurrentLimit)) {
					io.changeMainConfig(config -> {
						config.CurrentLimits.StatorCurrentLimit = maxStator.in(Amps);
						config.CurrentLimits.SupplyCurrentLimit = maxSupply.in(Amps);
						return config;
					});
				}
				io.setVelocitySetpoint(velocitySetpoint, slot);
				return io;
			};
			return new Setpoint(applier, Mode.VELOCITY, velocitySetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a velocity setpoint to go to a velocity with voltage limits.
		 *
		 * @param velocitySetpoint Velocity to go to in mechanism units.
		 * @param peakForward      Peak forward voltage.
		 * @param peakReverse      Peak reverse voltage.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVelocitySetpointAndVoltageLimit(
				AngularVelocity velocitySetpoint, Voltage peakForward, Voltage peakReverse) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				TalonFXConfiguration voltage = io.getMotorIOConfig();
				if (!Util.epsilonEquals(peakForward.in(Volts), voltage.Voltage.PeakForwardVoltage)) {
					io.changeMainConfig(config -> {
						config.Voltage.PeakForwardVoltage = peakForward.in(Volts);
						config.Voltage.PeakReverseVoltage = peakReverse.in(Volts);
						return config;
					});
					io.changeFollowerConfig(config -> {
						config.Voltage.PeakForwardVoltage = peakForward.in(Volts);
						config.Voltage.PeakReverseVoltage = peakReverse.in(Volts);
						return config;
					});
				}
				io.setVelocitySetpoint(velocitySetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.VELOCITY, velocitySetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a velocity setpoint to go to a velocity with voltage limits.
		 *
		 * @param velocitySetpoint Velocity to go to in mechanism units.
		 * @param peakForward      Peak forward voltage.
		 * @param peakReverse      Peak reverse voltage.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVelocitySetpointAndVoltageLimit(
				AngularVelocity velocitySetpoint, int slot, Voltage peakForward, Voltage peakReverse) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				TalonFXConfiguration voltage = io.getMotorIOConfig();
				if (!Util.epsilonEquals(peakForward.in(Volts), voltage.Voltage.PeakForwardVoltage)) {
					io.changeMainConfig(config -> {
						config.Voltage.PeakForwardVoltage = peakForward.in(Volts);
						config.Voltage.PeakReverseVoltage = peakReverse.in(Volts);
						return config;
					});
					io.changeFollowerConfig(config -> {
						config.Voltage.PeakForwardVoltage = peakForward.in(Volts);
						config.Voltage.PeakReverseVoltage = peakReverse.in(Volts);
						return config;
					});
				}
				io.setVelocitySetpoint(velocitySetpoint, slot);
				return io;
			};
			return new Setpoint(applier, Mode.VELOCITY, velocitySetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to run at a percent of maximum voltage.
		 *
		 * @param percent Percent to run at.
		 * @return A new Setpoint.
		 */
		public static Setpoint withDutyCycleSetpoint(Dimensionless percent) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setDutyCycleSetpoint(percent);
				return io;
			};
			return new Setpoint(applier, Mode.DUTY_CYCLE, percent.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to idle.
		 *
		 * @return A new Setpoint.
		 */
		public static Setpoint withNeutralSetpoint() {
			return NEUTRAL;
		}

		/**
		 * Creates a setpoint to coast.
		 *
		 * @return A new Setpoint.
		 */
		public static Setpoint withCoastSetpoint() {
			return COAST;
		}

		public void apply(MotorIO io) {
			applier.apply(io);
		}
	}
}
