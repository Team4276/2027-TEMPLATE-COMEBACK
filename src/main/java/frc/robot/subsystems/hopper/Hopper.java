package frc.robot.subsystems.hopper;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;

/** Hopper output presets, coordinated with Feeder by Superstructure's feed and idle commands. */
public class Hopper extends MotorSubsystem<MotorIO> {
    public static final Setpoint IDLE = Setpoint.withVoltageSetpoint(HopperConstants.kIdleVoltage);
    public static final Setpoint FEED = Setpoint.withVoltageSetpoint(HopperConstants.kFeedVoltage);
    public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(HopperConstants.kExhaustVoltage);

    public static final Hopper mInstance = new Hopper();

    public Hopper() {
        super(HopperConstants.getMotorIO(), "Hopper");
    }

}
