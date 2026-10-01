package frc.robot.subsystems.drive.scuffed;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Angle;


import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * One Talon-driven wheel and Spark-steered module. MotorIO exposes wheel/module angles after
 * adapter conversion; this layer alone converts wheel radians to distance using wheel radius.
 */
public class SwerveModule {
    public enum ModulePosition {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    public final MotorIO mDriveFx;
    public final MotorIO mTurnSpark;
    public final Angle mTurnOffset;
    private final double wheelRadiusMeters;

    public SwerveModule(MotorIOTalonFXConfig driveConfig, MotorIOSparkMaxConfig turnConfig, Angle turnOffset) {
        this(new MotorIOTalonFX(driveConfig), new MotorIOSparkMax(turnConfig),
                turnOffset, DriveConstants.wheelRadiusMeters);
    }

    SwerveModule(MotorIO drive, MotorIO turn, Angle turnOffset, double wheelRadiusMeters) {
        mDriveFx = drive;
        mTurnSpark = turn;
        mTurnOffset = turnOffset;
        this.wheelRadiusMeters = wheelRadiusMeters;
    }

    public void updateInputs() {
        mDriveFx.updateInputs();
        mTurnSpark.updateInputs();
    }

    public void setVelocity(SwerveModuleVelocity velocity) {
        // Subtract the absolute encoder's mounting offset to work in robot-relative wheel angles.
        Rotation2d currentAngle = new Rotation2d(mTurnSpark.getPosition().minus(mTurnOffset));
        // Reverse wheel direction when it saves steering travel, then reduce drive output while
        // steering is misaligned to avoid pushing sideways during the turn.
        velocity = velocity.optimize(currentAngle).cosineScale(currentAngle);
        mDriveFx.applySetpoint(Setpoint.withVelocitySetpoint(
                RotationsPerSecond.of(velocity.velocity / (2 * Math.PI * wheelRadiusMeters)), 0));
        // Restore the sensor-frame offset for the controller. Both swerve loops use slot 0,
        // overriding MotorIO's general-purpose default slots.
        mTurnSpark.applySetpoint(Setpoint.withPositionSetpoint(velocity.angle.getMeasure().plus(mTurnOffset), 0));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Meters.of(mDriveFx.getPosition().in(Radians) * wheelRadiusMeters),
                new Rotation2d(mTurnSpark.getPosition().minus(mTurnOffset)));
    }

    public SwerveModuleVelocity getVelocity() {
        return new SwerveModuleVelocity(
                MetersPerSecond.of(mDriveFx.getVelocity().in(RadiansPerSecond) * wheelRadiusMeters),
                new Rotation2d(mTurnSpark.getPosition().minus(mTurnOffset)));
    }
}
