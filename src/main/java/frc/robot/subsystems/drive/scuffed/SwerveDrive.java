package frc.robot.subsystems.drive.scuffed;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Angle;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Ports;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.scuffed.SwerveModule.ModulePosition;

public class SwerveDrive {
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.kModuleTranslations);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.ZERO, getModulePositions());
    private final Pigeon2 gyro = new Pigeon2(Ports.PIGEON.id, Ports.PIGEON.bus);

    public SwerveDrive(){
        modules = new SwerveModule[]{
            new SwerveModule(DriveConstants.getDriveIOConfig(Ports.FRONT_LEFT_DRIVE), DriveConstants.getTurnIOConfig(Ports.FRONT_LEFT_TURN)),
            new SwerveModule(DriveConstants.getDriveIOConfig(Ports.FRONT_RIGHT_DRIVE), DriveConstants.getTurnIOConfig(Ports.FRONT_RIGHT_TURN)),
            new SwerveModule(DriveConstants.getDriveIOConfig(Ports.BACK_LEFT_DRIVE), DriveConstants.getTurnIOConfig(Ports.BACK_LEFT_TURN)),
            new SwerveModule(DriveConstants.getDriveIOConfig(Ports.BACK_RIGHT_DRIVE), DriveConstants.getTurnIOConfig(Ports.BACK_RIGHT_TURN)),
        };
    }

    public void updateTelemetry() {
        odometry.update(new Rotation2d(getGyroAngle()), getModulePositions());
    }

    public Pose2d getPose(){
        return odometry.getPose();
    }

    public Angle getGyroAngle(){
        return gyro.getYaw(true).getValue();
    }

    public ChassisVelocities getRobotRelativeVelocity() {
        return kinematics.toChassisVelocities(getModuleVelocities());
    }

    public ChassisVelocities getFieldRelativeVelocity() {
        return getFieldRelativeVelocity().toFieldRelative(new Rotation2d(getGyroAngle()));
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }

        return positions;
    }

    public SwerveModuleVelocity[] getModuleVelocities() {
        SwerveModuleVelocity[] velocities = new SwerveModuleVelocity[4];

        for (int i = 0; i < 4; i++) {
            velocities[i] = modules[i].getVelocity();
        }

        return velocities;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPose(pose);
    }

    public void setFieldRelativeChassisVelocities(ChassisVelocities velocities) {
        setRobotRelativeChassisVelocities(velocities.toFieldRelative(new Rotation2d(getGyroAngle())));
    }

    public void setRobotRelativeChassisVelocities(ChassisVelocities velocities) {
        var moduleVelocities = kinematics.toSwerveModuleVelocities(velocities);

        for (int i = 0; i < 4; i++) {
            modules[i].setVelocity(moduleVelocities[i]);
        }
    }

    public SwerveModule getModule(ModulePosition module) {
        return modules[module.ordinal()];
    }
}
