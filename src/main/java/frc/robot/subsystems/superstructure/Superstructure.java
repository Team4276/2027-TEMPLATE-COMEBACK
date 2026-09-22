package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

public class Superstructure extends SubsystemBase {
    public static final Superstructure mInstance = new Superstructure();

    @Override
    public void periodic() {
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
