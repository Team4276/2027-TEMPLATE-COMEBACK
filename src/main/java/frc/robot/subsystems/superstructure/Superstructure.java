package frc.robot.subsystems.superstructure;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

import frc.lib.io.MotorIO.Setpoint;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheels.Flywheel;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeDeploy;
import frc.robot.subsystems.intake.IntakeRollers;

public class Superstructure extends SubsystemBase {
    public static final Superstructure mInstance = new Superstructure();

    @Override
    public void periodic() {
    }

    public Command setIntakeDeploy(Setpoint setpoint) {
        return IntakeDeploy.mInstance.setpointCommand(setpoint);
    }

    public Command setIntakeRollers(Setpoint setpoint) {
        return IntakeRollers.mInstance.setpointCommand(setpoint);
    }

    public Command shootHub() {
        return Flywheel.mInstance.setpointCommand(Flywheel.SHUB)
                .withName("Shoot Hub");
    }

    public Command shootHubFar() {
        return Flywheel.mInstance.setpointCommand(Flywheel.SHOWER)
                .withName("Shoot Hub Far");
    }

    public Command ferry() {
        return Flywheel.mInstance.setpointCommand(Flywheel.SHERRY)
                .withName("Ferry");
    }

    public Command feed() {
        return Commands.startEnd(
                () -> {
                    Feeder.mInstance.applySetpoint(Feeder.FEED);
                    Hopper.mInstance.applySetpoint(Hopper.FEED);
                },
                () -> {
                    Feeder.mInstance.applySetpoint(Feeder.IDLE);
                    Hopper.mInstance.applySetpoint(Hopper.IDLE);
                },
                Feeder.mInstance,
                Hopper.mInstance)
                .withName("Feed");
    }

    public Command idleFlywheels() {
        return Flywheel.mInstance.setpointCommand(Flywheel.IDLE)
                .withName("Idle Flywheel");
    }

    public Command idleFeeders() {
        return Feeder.mInstance.setpointCommand(Feeder.IDLE)
                .alongWith(Hopper.mInstance.setpointCommand(Hopper.IDLE))
                .withName("Idle Feeders");
    }
}
