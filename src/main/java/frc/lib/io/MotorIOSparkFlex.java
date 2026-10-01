package frc.lib.io;

import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.wpilib.units.AngleUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Voltage;
import org.wpilib.hardware.bus.CANPort;
// import org.wpilib.smartdashboard.SmartDashboard;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.UnaryOperator;

/**
 * REV Flex adapter using relative encoder feedback. Basic motor commands are implemented,
 * but CTRE configuration hooks used by MotorSubsystem tuning and limit-changing setpoints
 * are placeholders; configure REV settings through SparkFlexConfig instead.
 */
public class MotorIOSparkFlex extends MotorIO {
	protected final SparkFlex main;
	protected final SparkFlex[] followers;
	protected SparkFlexConfig config;
	protected SparkFlexConfig followerConfig;
	// A single worker keeps configuration transactions off the periodic robot thread.
	private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
	private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5,
			java.util.concurrent.TimeUnit.MILLISECONDS, queue);
	private boolean configFailed = false;
	private double positionFactor = 1.0;
	private double velocityFactor = 1.0;

	public void applyConfig(SparkFlex spark, SparkFlexConfig config) {
		threadPoolExecutor.submit(() -> {
			for (int i = 0; i < 5; i++) {
				REVLibError result = spark.configure(config, ResetMode.kResetSafeParameters,
						PersistMode.kPersistParameters);
				if (result == REVLibError.kOk) {
					break;
				} else {
					configFailed = true;
				}
			}
		});
	}

	@Override
	public void updateInputs() {
		inputs.enabled = getEnabled();
		inputs.setPointType = Mode.IDLE;
		inputs.setPointValueAsDouble = 0.0;

		// Convert native rotations/RPM to base units after applying software mechanism scales.
		inputs.position[0] = Rotations.of(main.getEncoder().getPosition().get() * positionFactor).baseUnitMagnitude();
		inputs.velocity[0] = RPM.of(main.getEncoder().getVelocity().get() * velocityFactor).baseUnitMagnitude();
		inputs.statorCurrent[0] = main.getOutputCurrent().get();
		// This adapter reports output current in both fields; it does not measure supply current.
		inputs.supplyCurrent[0] = main.getOutputCurrent().get();
		inputs.motorVoltage[0] = main.getBusVoltage().get() * main.getAppliedOutput().get();
		// Legacy exception to MotorIO's base-unit convention: Flex temperature is raw Celsius.
		inputs.motorTemperature[0] = main.getMotorTemperature().get();
		inputs.acceleration[0] = 0.0;

		for (int i = 0; i < followers.length; i++) {
			inputs.position[i + 1] = Rotations.of(followers[i].getEncoder().getPosition().get() * positionFactor).baseUnitMagnitude();
			inputs.velocity[i + 1] = RPM.of(followers[i].getEncoder().getVelocity().get() * velocityFactor).baseUnitMagnitude();
			inputs.statorCurrent[i + 1] = followers[i].getOutputCurrent().get();
			inputs.supplyCurrent[i + 1] = followers[i].getOutputCurrent().get();
			inputs.motorVoltage[i + 1] = followers[i].getBusVoltage().get() * followers[i].getAppliedOutput().get();
			inputs.motorTemperature[i + 1] = followers[i].getMotorTemperature().get();
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

		inputs.configFailed = false;
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
		threadPoolExecutor.submit(() -> {
			main.getEncoder().setPosition(mechanismPosition.div(positionFactor).in(Rotations));
		});
	}

	@Override
	public void zeroSensors() {
		setCurrentPosition(Units.Rotations.of(0.0));
	}

	private void setIdleMode(SparkFlex spark, IdleMode idleMode) {
		// SmartDashboard.putNumber("SPARK FLEX NEUTRAL MODE SET!!", Timer.getMonotonicTimestamp());
		threadPoolExecutor.submit(() -> {
			main.configure(config.idleMode(idleMode), ResetMode.kNoResetSafeParameters,
					PersistMode.kNoPersistParameters);
		});
	}

	@Override
	public void setNeutralBrake(boolean wantsBrake) {
		IdleMode idleMode = wantsBrake ? IdleMode.kBrake : IdleMode.kCoast;
		setIdleMode(main, idleMode);
		for (SparkFlex spark : followers) {
			setIdleMode(spark, idleMode);
		}
	}

	@Override
	public void useSoftLimits(boolean enable) {
		// Not implemented: MotorIO's soft-limit toggle has no Spark configuration translation.
	}

	@Override
	public TalonFXConfiguration getMotorIOConfig() {
		// Compatibility placeholder, not a readback of the Spark's settings.
		return new TalonFXConfiguration();
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * Applies a SparkFlexConfig to the main motor.
	 *
	 * @param configuration Configuration to apply.
	 */
	public void setMainConfig(SparkFlexConfig configuration) {
		config = configuration;
		applyConfig(main, config);
	}

	public void setMainConfig(TalonFXConfiguration configuration) {
		// CTRE settings cannot be applied to REV hardware; use the SparkFlexConfig overload.
	}

	/**
	 * Unsupported CTRE configuration hook; the callback is not invoked for this adapter.
	 *
	 * @param configChanger Mutating operation to apply on the current
	 *                      configuration.
	 */
	public void changeMainConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		// Limit-changing MotorIO setpoints still send their target, but do not change REV limits.
	}

	/**
	 * Unsupported CTRE follower configuration hook; use SparkFlexConfig for REV settings.
	 *
	 * @param configChanger Mutating operation to apply on the current
	 *                      configuration.
	 */
	public void changeFollowerConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		// No CTRE-to-REV configuration translation is provided.
	}

	/**
	 * Creates a Spark Flex leader and followers from a provided configuration.
	 *
	 * @param config Device addresses, conversion factors, and REV settings.
	 */
	public MotorIOSparkFlex(MotorIOSparkFlexConfig config) {
		super(config.unit, config.time, config.followerIDs.length);
		main = new SparkFlex(config.canPort, config.mainID, MotorType.kBrushless);
		setMainConfig(config.mainConfig);
		positionFactor = config.positionConversionFactor;
		velocityFactor = config.velocityConversionFactor;

		followerConfig = config.followerConfig;
		followers = new SparkFlex[config.followerIDs.length];
		for (int i = 0; i < config.followerIDs.length; i++) {
			followers[i] = new SparkFlex(config.canPort, config.followerIDs[i], MotorType.kBrushless);
			SparkFlexConfig motorConfig = new SparkFlexConfig();
			motorConfig.apply(followerConfig).follow(main, config.followerInverted[i]);
			applyConfig(followers[i], motorConfig);
		}
	}

	/**
	 * Configuration for this adapter. Default slots are MAXMotion 0, velocity 1, position PID 2.
	 * Conversion factors are software scales and should not duplicate device-side encoder scaling.
	 */
	public static class MotorIOSparkFlexConfig {
		public AngleUnit unit = Units.Rotations;
		public TimeUnit time = Units.Seconds;
		public double positionConversionFactor = 1.0;
		public double velocityConversionFactor = 1.0;
		public CANPort canPort = CANPort.CAN_S0;
		public int mainID = -1;
		public SparkFlexConfig mainConfig = new SparkFlexConfig();
		public int[] followerIDs = new int[0];
		public SparkFlexConfig followerConfig = new SparkFlexConfig();
		public boolean[] followerInverted = new boolean[0];
	}

	@Override
	public boolean getConfigFailed() {
		return configFailed;
	}

	public SparkFlex getMain() {
		return main;
	}
}
