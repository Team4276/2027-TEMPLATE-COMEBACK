package frc.robot.subsystems.flywheels;

import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;

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
