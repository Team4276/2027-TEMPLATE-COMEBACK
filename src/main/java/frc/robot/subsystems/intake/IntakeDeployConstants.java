package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.wpilib.units.Units;
import org.wpilib.units.measure.Voltage;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.robot.Ports;
import frc.robot.RobotConstants;

public class IntakeDeployConstants {
    public static final Voltage kIdleVoltage = Units.Volts.of(0.0);
    public static final Voltage kDeployVoltage = Units.Volts.of(6.0);
    public static final Voltage kStowVoltage = Units.Volts.of(-6.0);

    public static SparkMaxConfig getSparkConfig() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0)
                .inverted(true);
        config.signals
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        return config;
    }

    public static MotorIOSparkMaxConfig getIOConfig() {
        MotorIOSparkMaxConfig config = new MotorIOSparkMaxConfig();
        config.unit = Units.Rotations;
        config.time = Units.Minutes;
        config.mainID = Ports.INTAKE_DEPLOY.id;
        config.canPort = Ports.INTAKE_DEPLOY.canPort;
        config.mainConfig = getSparkConfig();
        return config;
    }

    public static MotorIO getMotorIO() {
        return switch (RobotConstants.mode) {
            case REAL -> new MotorIOSparkMax(getIOConfig());
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
}
