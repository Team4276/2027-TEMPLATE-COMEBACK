package frc.robot.subsystems.superstructure;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheels.Flywheel;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeDeploy;
import frc.robot.subsystems.intake.IntakeRollers;

/**
 * Composes mechanism requests into driver actions. Child commands own the actual motor subsystem
 * requirements. Shooter presets latch until stopped; intake and feed actions stop on release.
 */
public class Superstructure extends SubsystemBase {
    public static final Superstructure mInstance = new Superstructure();

    @Override
    public void periodic() {
    }

    private MotorSubsystem<?>[] getMechanisms() {
        // Resolve every singleton even when bring-up mode skips their button bindings.
        // Unconstructed subsystems cannot send a neutral request to their motor controllers.
        return new MotorSubsystem<?>[] {
                Flywheel.mInstance, Feeder.mInstance, Hopper.mInstance,
                IntakeDeploy.mInstance, IntakeRollers.mInstance
        };
    }

    /** Clears persistent requests before a new enabled period. */
    public void stopAll() {
        for (MotorSubsystem<?> mechanism : getMechanisms()) {
            mechanism.stop();
        }
    }

    /** Inhibits mechanism output during drivetrain bring-up, even if another command requests it. */
    public void disableAll() {
        for (MotorSubsystem<?> mechanism : getMechanisms()) {
            mechanism.stop();
            mechanism.disable();
        }
    }

    public Command idleIntake() {
        return setIntake(IntakeDeploy.IDLE, IntakeRollers.IDLE)
                .withName("Idle Intake");
    }

    public Command deployIntake() {
        // The driver's deploy action also collects; release stops pivot voltage and rollers.
        return setIntake(IntakeDeploy.DEPLOY, IntakeRollers.INTAKE)
                .withName("Deploy Intake");
    }

    public Command runIntake() {
        return setIntake(IntakeDeploy.IDLE, IntakeRollers.INTAKE)
                .withName("Run Intake");
    }

    public Command exhaustIntake() {
        return setIntake(IntakeDeploy.IDLE, IntakeRollers.EXHAUST)
                .withName("Exhaust Intake");
    }

    public Command retractIntake() {
        return setIntake(IntakeDeploy.STOW, IntakeRollers.IDLE)
                .withName("Stow Intake");
    }

    private Command setIntake(Setpoint deploy, Setpoint rollers) {
        // Deploy is voltage-controlled with no automatic endpoint detection. Releasing the
        // button (or interrupting this group) must stop both motors instead of latching voltage.
        return IntakeDeploy.mInstance.holdSetpointCommand(deploy)
                .alongWith(IntakeRollers.mInstance.holdSetpointCommand(rollers));
    }

    public Command shootHub() {
        // Skip the entire spin-up group if feed was already requested. This identity check
        // depends on callers using Feeder.FEED, not a newly constructed equivalent setpoint.
        return Flywheel.mInstance.setpointCommand(Flywheel.SHUB)
                .alongWith(Feeder.mInstance.setpointCommand(Feeder.SPINUP)
                        .alongWith(Hopper.mInstance.setpointCommand(Hopper.EXHAUST)))
                .unless(() -> Feeder.mInstance.getSetpoint() == Feeder.FEED)
                .withName("Shoot Hub");
    }

    public Command shootTower() {
        return Flywheel.mInstance.setpointCommand(Flywheel.SHOWER)
                .alongWith(Feeder.mInstance.setpointCommand(Feeder.SPINUP)
                        .alongWith(Hopper.mInstance.setpointCommand(Hopper.EXHAUST)))
                .unless(() -> Feeder.mInstance.getSetpoint() == Feeder.FEED)
                .withName("Shoot Tower");
    }

    public Command ferry() {
        return Flywheel.mInstance.setpointCommand(Flywheel.SHERRY)
                .withName("Ferry");
    }

    public Command feed() {
        // Feeding is driver-controlled; this command does not wait for Flywheel.spunUp().
        return Feeder.mInstance.holdSetpointCommand(Feeder.FEED)
                .alongWith(Hopper.mInstance.holdSetpointCommand(Hopper.FEED))
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
