package frc.robot.subsystems.feeder;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Voltage;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOSparkFlex;
import frc.lib.io.MotorIOSparkFlex.MotorIOSparkFlexConfig;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.RobotConstants;

public class FeederConstants {
	public static final Voltage kIdleVoltage = Units.Volts.of(0.0);
	public static final Voltage kSpinupVoltage = Units.Volts.of(12.0);
	public static final Voltage kFeedVoltage = Units.Volts.of(8.0);
	public static final Voltage kExhaustVoltage = Units.Volts.of(-12.0);

	public static SparkFlexConfig getSparkConfig() {
		SparkFlexConfig config = new SparkFlexConfig();
		
        config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0)
                .inverted(true)
                .openLoopRampRate(1.0)
                .closedLoopRampRate(1.0);
        config.signals
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

		return config;
	}

	public static MotorIOSparkFlexConfig getIOConfig() {
		MotorIOSparkFlexConfig config = new MotorIOSparkFlexConfig();
		config.unit = Units.Rotations;
		config.time = Units.Minutes;
		config.mainID = Ports.FEEDER.id;
		config.canPort = Ports.FEEDER.canPort;
		config.mainConfig = getSparkConfig();
		return config;
	}

	public static MotorIO getMotorIO() {
		return switch (RobotConstants.mode) {
			case REAL -> new MotorIOSparkFlex(getIOConfig());
			case SIM -> new MotorIO(Units.Rotations, Units.Minutes) {
				@Override
				public void updateInputs() {};
			};
			case REPLAY -> new MotorIO(Units.Rotations, Units.Minutes) {
				@Override
				public void updateInputs() {
				};
			};
		};
	}

	public static RollerSimConstants getSimConstants() {
		RollerSimConstants simConstants = new RollerSimConstants();

		simConstants.motor = DCMotor.getNeoVortex(1);
		simConstants.gearing = 1.0;
		simConstants.momentOfInertia = 0.01;

		return simConstants;
	}
}
