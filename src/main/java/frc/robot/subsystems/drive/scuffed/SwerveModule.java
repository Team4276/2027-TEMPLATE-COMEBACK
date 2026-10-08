package frc.robot.subsystems.drive.scuffed;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;


import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIO;
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

    public final MotorIO mDriveFx;
    public final MotorIO mTurnSpark;
    public final Angle mTurnOffset;
    private final double wheelRadiusMeters;
    private int id;

    public SwerveModule(MotorIOTalonFXConfig driveConfig, MotorIOSparkMaxConfig turnConfig, Angle turnOffset) {
        id = driveConfig.mainID;
        this(new MotorIOTalonFX(driveConfig), new MotorIOSparkMax(turnConfig),
                turnOffset, DriveConstants.wheelRadiusMeters);
    }

    SwerveModule(MotorIO drive, MotorIO turn, Angle turnOffset, double wheelRadiusMeters) {
        id = -1;
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
        velocity = velocity.optimize(getPosition().angle);
        mDriveFx.applySetpoint(Setpoint.withVelocitySetpoint(
                RotationsPerSecond.of(velocity.velocity / (2 * Math.PI * wheelRadiusMeters)), 0));

        var turnPosition = new Rotation2d(velocity.angle.getMeasure().plus(mTurnOffset)).getMeasure();

        turnPosition = Rotations.of(MathUtil.inputModulus(velocity.angle.getMeasure().plus(mTurnOffset).in(Rotations), 0.0, 1.0));
        Logger.recordOutput("Turn " + id, turnPosition.in(Rotations));
        mTurnSpark.applySetpoint(Setpoint.withPositionSetpoint(turnPosition, 0));
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
