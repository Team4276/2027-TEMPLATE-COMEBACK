package frc.robot.subsystems.flywheels;

import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;

/**
 * Shooter speed presets for the paired flywheel motors. Explicit slot 0 matches FlywheelConstants;
 * the general MotorIO velocity default is slot 1. Readiness checks come from FlywheelMotorSubsystem.
 */
public class Flywheel extends FlywheelMotorSubsystem<MotorIO> {
    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    public static final Setpoint SHOWER = Setpoint.withVelocitySetpoint(FlywheelConstants.kShowerVelocity, 0);
    public static final Setpoint SHUB = Setpoint.withVelocitySetpoint(FlywheelConstants.kShubVelocity, 0);
    public static final Setpoint SHERRY = Setpoint.withVelocitySetpoint(FlywheelConstants.kSherryVelocity, 0);

    public static final Flywheel mInstance = new Flywheel();

    public Flywheel() {
        super(
                FlywheelConstants.getMotorIO(),
                "Flywheel",
                FlywheelConstants.kEpsilonThreshold,
                FlywheelConstants.kDebounceTime);
    }
}
