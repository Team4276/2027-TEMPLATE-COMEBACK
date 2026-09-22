package frc.robot.subsystems.examplesubsystem;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;

public class ExampleSubsystem extends MotorSubsystem<MotorIO> {
    public static final Setpoint EXAMPLE_SETPOINT = Setpoint
            .withVoltageSetpoint(ExampleSubsystemConstants.kExampleVoltage);
    public static final Setpoint IDLE = Setpoint.withVoltageSetpoint(ExampleSubsystemConstants.kIdleVoltage);

    public static final ExampleSubsystem mInstance = new ExampleSubsystem();

    public ExampleSubsystem() {
        super(ExampleSubsystemConstants.getMotorIO(), "Example Subsystem");
    }

}
