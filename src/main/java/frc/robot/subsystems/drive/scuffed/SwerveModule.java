package frc.robot.subsystems.drive.scuffed;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.AngularVelocity;

import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.robot.subsystems.drive.DriveConstants;

public class SwerveModule {
    public enum ModulePosition {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    public final MotorIOTalonFX mDriveFx;
    public final MotorIOSparkMax mTurnSpark;

    public SwerveModule(MotorIOTalonFXConfig driveConfig, MotorIOSparkMaxConfig turnConfig) {
        mDriveFx = new MotorIOTalonFX(driveConfig);
        mTurnSpark = new MotorIOSparkMax(turnConfig);
    }

    public void setVelocity(SwerveModuleVelocity velocity) {
        mDriveFx.applySetpoint(Setpoint.withVelocitySetpoint(AngularVelocity
                .ofBaseUnits(velocity.velocity / (2 * Math.PI * DriveConstants.wheelRadiusMeters),
                        RotationsPerSecond)));
        mTurnSpark.applySetpoint(Setpoint.withPositionSetpoint(velocity.angle.getMeasure()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Meters.of(mDriveFx.getPosition().in(Radians) * DriveConstants.wheelRadiusMeters),
                new Rotation2d(mTurnSpark.getPosition()));
    }

    public SwerveModuleVelocity getVelocity() {
        return new SwerveModuleVelocity(
                MetersPerSecond.of(mDriveFx.getVelocity().in(RadiansPerSecond) * DriveConstants.wheelRadiusMeters),
                new Rotation2d(mTurnSpark.getPosition()));
    }
}
