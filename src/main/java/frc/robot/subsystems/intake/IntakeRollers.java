package frc.robot.subsystems.intake;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;

public class IntakeRollers extends MotorSubsystem<MotorIO> {
    public static final Setpoint IDLE = Setpoint.withVoltageSetpoint(IntakeRollersConstants.kIdleVoltage);
    public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(IntakeRollersConstants.kIntakeVoltage);
    public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(IntakeRollersConstants.kExhaustVoltage);

    public static final IntakeRollers mInstance = new IntakeRollers();

    public IntakeRollers() {
        super(IntakeRollersConstants.getMotorIO(), "IntakeRollers");
    }
}
