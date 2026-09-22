package frc.robot.subsystems.superstructure;

import org.wpilib.command2.Command;
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

    public Command idleIntake() {
        return setIntake(IntakeDeploy.IDLE, IntakeRollers.IDLE)
                .withName("Idle Intake");
    }

    public Command deployIntake() {
        return setIntake(IntakeDeploy.DEPLOY, IntakeRollers.IDLE)
                .withName("Idle Intake");
    }

    public Command runIntake() {
        return setIntake(IntakeDeploy.IDLE, IntakeRollers.INTAKE)
                .withName("Idle Intake");
    }

    public Command exhaustIntake() {
        return setIntake(IntakeDeploy.IDLE, IntakeRollers.INTAKE)
                .withName("Idle Intake");
    }

    public Command retractIntake() {
        return setIntake(IntakeDeploy.STOW, IntakeRollers.IDLE)
                .withName("Idle Intake");
    }

    private Command setIntake(Setpoint deploy, Setpoint rollers) {
        return IntakeDeploy.mInstance.setpointCommand(deploy)
                .alongWith(IntakeRollers.mInstance.setpointCommand(rollers))
                .withName("Idle Intake");
    }

    public Command shootHub() {
        return Flywheel.mInstance.setpointCommand(Flywheel.SHUB)
                .alongWith(Feeder.mInstance.setpointCommand(Feeder.SPINUP)
                        .alongWith(Hopper.mInstance.setpointCommand(Hopper.EXHAUST)))
                .unless(() -> Feeder.mInstance.getSetpoint() == Feeder.FEED)
                .withName("Shoot Hub");
    }

    public Command shootHubFar() {
        return Flywheel.mInstance.setpointCommand(Flywheel.SHOWER)
                .alongWith(Feeder.mInstance.setpointCommand(Feeder.SPINUP)
                        .alongWith(Hopper.mInstance.setpointCommand(Hopper.EXHAUST)))
                .unless(() -> Feeder.mInstance.getSetpoint() == Feeder.FEED)
                .withName("Shoot Hub Far");
    }

    public Command ferry() {
        return Flywheel.mInstance.setpointCommand(Flywheel.SHERRY)
                .withName("Ferry");
    }

    public Command feed() {
        return Feeder.mInstance.setpointCommand(Feeder.FEED)
                .alongWith(Hopper.mInstance.setpointCommand(Hopper.FEED))
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
