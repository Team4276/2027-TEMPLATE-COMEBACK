package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.subsystems.examplesubsystem.ExampleSubsystem;

public class Superstructure extends SubsystemBase {
    public static final Superstructure mInstance = new Superstructure();

    @Override
    public void periodic() {
    }

    public Command setOn() {
        return ExampleSubsystem.mInstance.setpointCommand(ExampleSubsystem.EXAMPLE_SETPOINT);
    }

    public Command setOff() {
        return ExampleSubsystem.mInstance.setpointCommand(ExampleSubsystem.IDLE);
    }

    public Command exampleCommand() { // TODO: check if works on button hold
        return ExampleSubsystem.mInstance.setpointCommand(ExampleSubsystem.EXAMPLE_SETPOINT)
                .finallyDo(() -> {
                    ExampleSubsystem.mInstance.applySetpoint(ExampleSubsystem.IDLE);
                })
                .withName("Example Command");
    }

    public Command testCommand() {
        return Commands.runEnd(() -> {
            Logger.recordOutput("Superstructure/Test", true);
        },
                () -> {
                    Logger.recordOutput("Superstructure/Test", false);
                })
                .withName("Test Command");
    }
}
