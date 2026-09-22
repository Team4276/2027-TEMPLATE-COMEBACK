package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Voltage;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class IntakeRollersConstants {
    public static final Voltage kIdleVoltage = Units.Volts.of(0.0);
    public static final Voltage kIntakeVoltage = Units.Volts.of(12.0);
    public static final Voltage kExhaustVoltage = Units.Volts.of(-12.0);

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 40.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return config;
    }

    public static MotorIOTalonFXConfig getIOConfig() {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.unit = Units.Rotations;
        config.time = Units.Minutes;
        config.mainID = Ports.INTAKE_ROLLERS.id;
        config.mainBus = Ports.INTAKE_ROLLERS.bus;
        config.mainConfig = getFXConfig();
        return config;
    }

    public static MotorIO getMotorIO() {
        return switch (RobotConstants.mode) {
            case REAL -> new MotorIOTalonFX(getIOConfig());
            case SIM -> new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
            case REPLAY -> new MotorIO(Units.Rotations, Units.Minutes) {
                @Override
                public void updateInputs() {
                }
            };
        };
    }

    public static RollerSimConstants getSimConstants() {
        RollerSimConstants simConstants = new RollerSimConstants();

        simConstants.motor = DCMotor.getKrakenX60(1);
        simConstants.gearing = 1.0;
        simConstants.momentOfInertia = 0.01;

        return simConstants;
    }
}
