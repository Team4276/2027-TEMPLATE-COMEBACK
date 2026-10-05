package frc.robot.subsystems.conveyor;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.subsystems.feeder.Feeder;

public class Conveyor extends MotorSubsystem<MotorIO> {
    public static final Setpoint IDLE = Setpoint.withVoltageSetpoint(ConveyorConstants.kIdleVoltage);
    public static final Setpoint SPINUP = Setpoint.withVoltageSetpoint(ConveyorConstants.kSpinupVoltage);
    public static final Setpoint FEED = Setpoint.withVoltageSetpoint(ConveyorConstants.kFeedVoltage);
    public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(ConveyorConstants.kExhaustVoltage);

    public static final Conveyor mInstance = new Conveyor();

    public Conveyor() {
        super(ConveyorConstants.getMotorIO(), "Conveyor");
    }

}
