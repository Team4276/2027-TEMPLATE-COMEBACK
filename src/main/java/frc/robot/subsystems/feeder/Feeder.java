package frc.robot.subsystems.feeder;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;

public class Feeder extends MotorSubsystem<MotorIO> {
    public static final Setpoint IDLE = Setpoint.withVoltageSetpoint(FeederConstants.kIdleVoltage);
    public static final Setpoint SPINUP = Setpoint.withVoltageSetpoint(FeederConstants.kSpinupVoltage);
    public static final Setpoint FEED = Setpoint.withVoltageSetpoint(FeederConstants.kFeedVoltage);
    public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(FeederConstants.kExhaustVoltage);

    public static final Feeder mInstance = new Feeder();

    public Feeder() {
        super(FeederConstants.getMotorIO(), "Feeder");
    }

}
