package frc.robot.subsystems.conveyor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Time;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.io.MotorIOTalonFX.ControlRequestGetter;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ConveyorConstants {
    public static final double kGearing = 1.0;

    // TODO: acceptable velocity error before considered "at speed"
    public static final AngularVelocity kEpsilonThreshold = Units.RPM.of(50.0);

    // TODO: debounce time for spunUpDebounced()
    public static final Time kDebounceTime = Units.Seconds.of(0.2);

    public static final AngularVelocity kConveyVelocity = Units.RPM.of(675.0);
    public static final AngularVelocity kInversConveyVelocity = Units.RPM.of(-675.0);

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 40.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 10.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.Slot0.kP = 0.1;
        config.Slot0.kV = 0.12;

        config.Feedback.SensorToMechanismRatio = kGearing;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return config;
    }

    public static MotorIOTalonFXConfig getIOConfig() {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.unit = Units.Rotations;
        config.time = Units.Minutes;
        config.mainID = Ports.CONVEYOR.id;
        config.mainBus = Ports.CONVEYOR.bus;
        config.mainConfig = getFXConfig();
        config.requestGetter = new ControlRequestGetter() {
            @Override
            public VelocityVoltage getVelocityRequest(AngularVelocity velocity, int slot) {
                return new VelocityVoltage(velocity).withSlot(slot).withEnableFOC(true);
            }
        };
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
        simConstants.gearing = kGearing;
        // TODO: moment of inertia (kg*m^2)
        simConstants.momentOfInertia = 0.01;

        return simConstants;
    }
}
