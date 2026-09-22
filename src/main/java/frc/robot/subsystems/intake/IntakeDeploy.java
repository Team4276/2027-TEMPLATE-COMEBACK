package frc.robot.subsystems.intake;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;

public class IntakeDeploy extends MotorSubsystem<MotorIO> {
    public static final Setpoint IDLE = Setpoint.withVoltageSetpoint(IntakeDeployConstants.kIdleVoltage);
    public static final Setpoint DEPLOY = Setpoint.withVoltageSetpoint(IntakeDeployConstants.kDeployVoltage);
    public static final Setpoint STOW = Setpoint.withVoltageSetpoint(IntakeDeployConstants.kStowVoltage);

    public static final IntakeDeploy mInstance = new IntakeDeploy();

    public IntakeDeploy() {
        super(IntakeDeployConstants.getMotorIO(), "IntakeDeploy");
    }
}
