package frc.lib.io;

import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.wpilib.units.AngleUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Voltage;
import org.wpilib.hardware.bus.CANPort;
// import org.wpilib.system.Timer;
// import org.wpilib.smartdashboard.SmartDashboard;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.UnaryOperator;

/**
 * REV adapter for a leader and followers, with relative or absolute feedback on the leader.
 * Basic setpoints work through MotorIO; TalonFX configuration mutations and live PID tuning
 * through MotorSubsystem are not implemented here. Use SparkMaxConfig for REV configuration.
 */
public class MotorIOSparkMax extends MotorIO {
	protected final SparkMax main;
	protected final SparkMax[] followers;
	protected SparkMaxConfig config;
	protected SparkMaxConfig followerConfig;
	// Serialize potentially blocking configuration writes off the robot loop. Submitting a
	// change does not mean it has reached the device when this method returns.
	private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
	private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5,
			java.util.concurrent.TimeUnit.MILLISECONDS, queue);
	// Latched after all retries fail; retained until robot code restarts.
	private volatile boolean configFailed = false;
	private boolean useAbsoluteEncoder = false;
	private double positionFactor = 1.0;
	private double velocityFactor = 1.0;

	public void applyConfig(SparkMax spark, SparkMaxConfig config) {
		threadPoolExecutor.submit(() -> {
			for (int i = 0; i < 5; i++) {
				REVLibError result = spark.configure(config, ResetMode.kResetSafeParameters,
						PersistMode.kPersistParameters);
				if (result == REVLibError.kOk) {
					break;
				} else if (i == 4) {
					configFailed = true;
				}
			}
		});
	}

	@Override
	public void updateInputs() {
		inputs.enabled = getEnabled();
		inputs.connected = main.getBusVoltage().isValid()
				&& (useAbsoluteEncoder ? main.getAbsoluteEncoder().getPosition().isValid()
						: main.getEncoder().getPosition().isValid());
		inputs.setPointType = Mode.IDLE;
		inputs.setPointValueAsDouble = 0.0;

		// REV values enter as rotations/RPM. Apply the software mechanism scale, then convert
		// to MotorIO's radians/rad/s. Setpoint methods undo this scale on the way out.
		if (!useAbsoluteEncoder) {
			inputs.position[0] = Rotations.of(main.getEncoder().getPosition().get() * positionFactor).baseUnitMagnitude();
			inputs.velocity[0] = RPM.of(main.getEncoder().getVelocity().get() * velocityFactor).baseUnitMagnitude();
		} else {
			inputs.position[0] = Rotations.of(main.getAbsoluteEncoder().getPosition().get() * positionFactor).baseUnitMagnitude();
			inputs.velocity[0] = RPM.of(main.getAbsoluteEncoder().getVelocity().get() * velocityFactor).baseUnitMagnitude();
		}
		// Supply current is currently a copy of output current, not a separate bus measurement.
		inputs.statorCurrent[0] = main.getOutputCurrent().get();
		inputs.supplyCurrent[0] = main.getOutputCurrent().get();
		inputs.motorVoltage[0] = main.getBusVoltage().get() * main.getAppliedOutput().get();
		inputs.motorTemperature[0] = Units.Celsius.of(main.getMotorTemperature().get()).baseUnitMagnitude();
		inputs.acceleration[0] = 0.0;

		for (int i = 0; i < followers.length; i++) {
			inputs.position[i + 1] = Rotations.of(followers[i].getEncoder().getPosition().get() * positionFactor).baseUnitMagnitude();
			inputs.velocity[i + 1] = RPM.of(followers[i].getEncoder().getVelocity().get() * velocityFactor).baseUnitMagnitude();
			inputs.statorCurrent[i + 1] = followers[i].getOutputCurrent().get();
			inputs.supplyCurrent[i + 1] = followers[i].getOutputCurrent().get();
			inputs.motorVoltage[i + 1] = followers[i].getBusVoltage().get() * followers[i].getAppliedOutput().get();
			inputs.motorTemperature[i + 1] = Units.Celsius.of(followers[i].getMotorTemperature().get()).baseUnitMagnitude();
			inputs.acceleration[i + 1] = 0.0;
		}

		inputs.pidVoltage = 0.0;

		// inputs.position[0] = Units.Rotations.of(main.getEncoder().getPosition());
		// inputs.velocity[0] =
		// Units.RotationsPerSecond.of(main.getEncoder().getVelocity());
		// inputs.statorCurrent[0] = Units.Amps.of(main.getOutputCurrent());
		// inputs.supplyCurrent[0] = Units.Amps.of(main.getOutputCurrent());
		// inputs.motorVoltage[0] = Units.Volts.of(main.getBusVoltage() *
		// main.getAppliedOutput());
		// inputs.motorTemperature[0] = Units.Celsius.of(main.getMotorTemperature());
		// inputs.acceleration[0] = Units.RotationsPerSecondPerSecond.of(0.0);

		// for (int i = 0; i < followers.length; i++) {
		// inputs.position[i + 1] =
		// Units.Rotations.of(followers[i].getEncoder().getPosition());
		// inputs.velocity[i + 1] =
		// Units.RotationsPerSecond.of(followers[i].getEncoder().getVelocity());
		// inputs.statorCurrent[i + 1] = Units.Amps.of(followers[i].getOutputCurrent());
		// inputs.supplyCurrent[i + 1] = Units.Amps.of(followers[i].getOutputCurrent());
		// inputs.motorVoltage[i + 1] = Units.Volts.of(followers[i].getBusVoltage() *
		// followers[i].getAppliedOutput());
		// inputs.motorTemperature[i + 1] =
		// Units.Celsius.of(followers[i].getMotorTemperature());
		// inputs.acceleration[i + 1] = Units.RotationsPerSecondPerSecond.of(0.0);
		// }

		// inputs.pidVoltage = Units.Volts.of(0.0);

		inputs.configFailed = configFailed;
	}

	@Override
	public void setNeutralSetpoint() {
		main.stopMotor();
	}

	@Override
	public void setCoastSetpoint() {
		threadPoolExecutor.submit(() -> {
			main.configure(config.idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters,
					PersistMode.kNoPersistParameters);
		});
	}

	@Override
	protected void setVoltageSetpoint(Voltage voltage) {
		main.setVoltage(voltage);
	}

	@Override
	protected void setDutyCycleSetpoint(Dimensionless percent) {
		main.setThrottle(percent.baseUnitMagnitude());
	}

	@Override
	protected void setMotionMagicSetpoint(Angle mechanismPosition) {
		setMotionMagicSetpoint(mechanismPosition, 0);
	}

	@Override
	protected void setMotionMagicSetpoint(Angle mechanismPosition, int slot) {
		main.getClosedLoopController().setSetpoint(mechanismPosition.div(positionFactor).in(Rotations),
				ControlType.kMAXMotionPositionControl, ClosedLoopSlot.fromInt(slot));
	}

	@Override
	protected void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
		setVelocitySetpoint(mechanismVelocity, 1);
	}

	@Override
	protected void setVelocitySetpoint(AngularVelocity mechanismVelocity, int slot) {
		main.getClosedLoopController().setSetpoint(mechanismVelocity.div(velocityFactor).in(RPM), ControlType.kVelocity,
				ClosedLoopSlot.fromInt(slot));
	}

	@Override
	protected void setPositionSetpoint(Angle mechanismPosition) {
		setPositionSetpoint(mechanismPosition, 2);
	}

	@Override
	protected void setPositionSetpoint(Angle mechanismPosition, int slot) {
		main.getClosedLoopController().setSetpoint(mechanismPosition.div(positionFactor).in(Rotations), ControlType.kPosition,
				ClosedLoopSlot.fromInt(slot));
	}

	@Override
	public void setCurrentPosition(Angle mechanismPosition) {
		// Only the relative encoder can be zeroed here. Absolute steering uses a mounting offset
		// in SwerveModule instead; selecting absolute feedback does not change this method.
		threadPoolExecutor.submit(() -> {
			main.getEncoder().setPosition(mechanismPosition.div(positionFactor).in(Rotations));
		});
	}

	@Override
	public void zeroSensors() {
		setCurrentPosition(Units.Rotations.of(0.0));
	}

	private void setIdleMode(SparkMax spark, IdleMode idleMode) {
		// SmartDashboard.putNumber("SPARK MAX NEUTRAL MODE SET!!",
		// Timer.getMonotonicTimestamp());
		threadPoolExecutor.submit(() -> {
			main.configure(config.idleMode(idleMode), ResetMode.kNoResetSafeParameters,
					PersistMode.kNoPersistParameters);
		});
	}

	@Override
	public void setNeutralBrake(boolean wantsBrake) {
		IdleMode idleMode = wantsBrake ? IdleMode.kBrake : IdleMode.kCoast;
		setIdleMode(main, idleMode);
		for (SparkMax spark : followers) {
			setIdleMode(spark, idleMode);
		}
	}

	@Override
	public void useSoftLimits(boolean enable) {
		// Not implemented: MotorIO's soft-limit toggle has no Spark configuration translation.
	}

	@Override
	public TalonFXConfiguration getMotorIOConfig() {
		// Compatibility placeholder only; this does not describe the Spark's current settings.
		return new TalonFXConfiguration();
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * Applies a SparkMaxConfig to the main motor.
	 *
	 * @param configuration Configuration to apply.
	 */
	public void setMainConfig(SparkMaxConfig configuration) {
		config = configuration;
		applyConfig(main, config);
	}

	public void setMainConfig(TalonFXConfiguration configuration) {
		// CTRE settings cannot be applied to REV hardware; use the SparkMaxConfig overload.
	}

	/**
	 * Unsupported CTRE configuration hook. The callback is not invoked for this adapter.
	 *
	 * @param configChanger Mutating operation to apply on the current
	 *                      configuration.
	 */
	public void changeMainConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		// Consequently, MotorIO setpoints with dynamic CTRE current/voltage limits do not
		// update those limits on a Spark; only their basic setpoint is applied.
	}

	/**
	 * Unsupported CTRE follower configuration hook; configure followers with SparkMaxConfig.
	 *
	 * @param configChanger Mutating operation to apply on the current
	 *                      configuration.
	 */
	public void changeFollowerConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		// No CTRE-to-REV configuration translation is provided.
	}

	/**
	 * Creates a Spark MAX leader and followers from a provided configuration.
	 *
	 * @param config Device addresses, feedback selection, and REV settings.
	 */
	public MotorIOSparkMax(MotorIOSparkMaxConfig config) {
		super(config.unit, config.time, config.followerIDs.length);
		main = new SparkMax(config.canPort, config.mainID, MotorType.kBrushless);
		setMainConfig(config.mainConfig);

		followerConfig = config.followerConfig;
		followers = new SparkMax[config.followerIDs.length];
		for (int i = 0; i < config.followerIDs.length; i++) {
			followers[i] = new SparkMax(config.canPort, config.followerIDs[i], MotorType.kBrushless);
			SparkMaxConfig motorConfig = new SparkMaxConfig();
			motorConfig.apply(followerConfig).follow(main, config.followerInverted[i]);
			applyConfig(followers[i], motorConfig);
		}

		useAbsoluteEncoder = config.useAbsoluteEncoder;
		positionFactor = config.positionConversionFactor;
		velocityFactor = config.velocityConversionFactor;
	}

	/**
	 * Configuration for this adapter. Default setpoint slots are MAXMotion 0, velocity 1,
	 * and position PID 2; explicit-slot setpoints override those defaults.
	 */
	public static class MotorIOSparkMaxConfig {
		public AngleUnit unit = Units.Rotations;
		public TimeUnit time = Units.Seconds;
		// Software scales, separate from REV's device-side encoder conversion settings.
		// Do not apply the same gear conversion both here and in SparkMaxConfig.
		public double positionConversionFactor = 1.0;
		public double velocityConversionFactor = 1.0;
		public boolean useAbsoluteEncoder = false;
		public CANPort canPort = CANPort.CAN_S0;
		public int mainID = -1;
		public SparkMaxConfig mainConfig = new SparkMaxConfig();
		public int[] followerIDs = new int[0];
		public SparkMaxConfig followerConfig = new SparkMaxConfig();
		public boolean[] followerInverted = new boolean[0];
	}

	@Override
	public boolean getConfigFailed() {
		return configFailed;
	}

	public SparkMax getMain() {
		return main;
	}
}
