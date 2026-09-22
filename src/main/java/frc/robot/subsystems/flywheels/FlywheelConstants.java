package frc.robot.subsystems.flywheels;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Time;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class FlywheelConstants {
    // TODO: gear ratio between motor and flywheel
    public static final double kGearing = 1.0;

    // TODO: acceptable velocity error before considered "at speed"
    public static final AngularVelocity kEpsilonThreshold = Units.RPM.of(50.0);

    // TODO: debounce time for spunUpDebounced()
    public static final Time kDebounceTime = Units.Seconds.of(0.2);

    public static final AngularVelocity kShowerVelocity = Units.RPM.of(2700.0);
    public static final AngularVelocity kShubVelocity = Units.RPM.of(2500.0);
    public static final AngularVelocity kSherryVelocity = Units.RPM.of(3000.0);

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        // TODO: stator current limit
        config.CurrentLimits.StatorCurrentLimit = 80.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        // TODO: supply current limit
        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 50.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.Slot0.kP = 0.0001;

        config.Feedback.SensorToMechanismRatio = kGearing;

        // TODO: motor inversion
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return config;
    }

    public static MotorIOTalonFXConfig getIOConfig() {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.unit = Units.Rotations;
        config.time = Units.Minutes;
        config.mainID = Ports.FLYWHEEL_LEFT.id;
        config.mainBus = Ports.FLYWHEEL_LEFT.bus;
        config.mainConfig = getFXConfig();
        config.followerIDs = new int[]{Ports.FLYWHEEL_RIGHT.id};
        config.followerConfig = getFXConfig();
        config.followerBuses = new CANBus[]{Ports.FLYWHEEL_RIGHT.bus};
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

        // TODO: motor type and count
        simConstants.motor = DCMotor.getKrakenX60(2);
        simConstants.gearing = kGearing;
        // TODO: moment of inertia (kg*m^2)
        simConstants.momentOfInertia = 0.01;

        return simConstants;
    }
}
