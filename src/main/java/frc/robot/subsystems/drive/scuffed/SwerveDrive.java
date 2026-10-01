package frc.robot.subsystems.drive.scuffed;

import java.util.function.Supplier;
import org.wpilib.math.geometry.Translation2d;
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

/**
 * Converts chassis motion into wheel commands and integrates wheel distances with gyro heading.
 * All module arrays use FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT order, matching kinematics.
 */
public class SwerveDrive {
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;

    private final SwerveDriveOdometry odometry;
    private final Supplier<Angle> gyroAngle;

    public SwerveDrive() {
        this(new SwerveModule[] {
                new SwerveModule(DriveConstants.getDriveIOConfig(Ports.FRONT_LEFT_DRIVE),
                        DriveConstants.getTurnIOConfig(Ports.FRONT_LEFT_TURN), DriveConstants.turnOffsets[0]),
                new SwerveModule(DriveConstants.getDriveIOConfig(Ports.FRONT_RIGHT_DRIVE),
                        DriveConstants.getTurnIOConfig(Ports.FRONT_RIGHT_TURN), DriveConstants.turnOffsets[1]),
                new SwerveModule(DriveConstants.getDriveIOConfig(Ports.BACK_LEFT_DRIVE),
                        DriveConstants.getTurnIOConfig(Ports.BACK_LEFT_TURN), DriveConstants.turnOffsets[2]),
                new SwerveModule(DriveConstants.getDriveIOConfig(Ports.BACK_RIGHT_DRIVE),
                        DriveConstants.getTurnIOConfig(Ports.BACK_RIGHT_TURN), DriveConstants.turnOffsets[3]),
        }, createGyroSupplier(), DriveConstants.kModuleTranslations);
    }

    private static Supplier<Angle> createGyroSupplier() {
        Pigeon2 gyro = new Pigeon2(Ports.PIGEON.id, Ports.PIGEON.bus);
        return () -> gyro.getYaw(true).getValue();
    }

    // Injection keeps geometry and odometry testable without constructing CAN devices.
    SwerveDrive(SwerveModule[] modules, Supplier<Angle> gyroAngle, Translation2d[] moduleTranslations) {
        this.modules = modules;
        this.gyroAngle = gyroAngle;
        kinematics = new SwerveDriveKinematics(moduleTranslations);
        updateModuleInputs();
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(getGyroAngle()), getModulePositions());
    }

    private void updateModuleInputs() {
        for (SwerveModule module : modules) {
            module.updateInputs();
        }
    }

    public void updateTelemetry() {
        updateModuleInputs();
        odometry.update(new Rotation2d(getGyroAngle()), getModulePositions());
    }

    public Pose2d getPose() {
        return odometry.getPose();
    }

    public Angle getGyroAngle() {
        return gyroAngle.get();
    }

    public ChassisVelocities getRobotRelativeVelocity() {
        return kinematics.toChassisVelocities(getModuleVelocities());
    }

    public ChassisVelocities getFieldRelativeVelocity() {
        return getRobotRelativeVelocity().toFieldRelative(getPose().getRotation());
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
        // Use the odometry heading so pose resets also change the driver/path reference frame.
        setRobotRelativeChassisVelocities(velocities.toRobotRelative(getPose().getRotation()));
    }

    public void setRobotRelativeChassisVelocities(ChassisVelocities velocities) {
        var moduleVelocities = kinematics.toSwerveModuleVelocities(velocities);
        // Scale all wheels together when any exceeds the limit, preserving their speed ratios.
        moduleVelocities = SwerveDriveKinematics.desaturateWheelVelocities(
                moduleVelocities, DriveConstants.kMaxVelocity);

        for (int i = 0; i < 4; i++) {
            modules[i].setVelocity(moduleVelocities[i]);
        }
    }

    public SwerveModule getModule(ModulePosition module) {
        return modules[module.ordinal()];
    }
}
