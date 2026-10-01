package frc.robot.subsystems.drive.scuffed;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.*;
import org.wpilib.math.kinematics.*;
import org.wpilib.units.measure.*;
import frc.lib.io.MotorIO;

/**
 * Exercises swerve math with injected motor/gyro feedback. These tests validate units, frame
 * changes, and command selection; they do not validate vendor firmware or physical calibration.
 */
class SwerveDriveTest {
    private static final double RADIUS = 0.05;

    private static class FakeMotor extends MotorIO {
        double position;
        double velocity;
        double requestedVelocity;
        double requestedPosition;
        int slot = -1;
        int updates;

        FakeMotor() { super(Rotations, Seconds); }
        @Override public void updateInputs() {
            inputs.position[0] = position;
            inputs.velocity[0] = velocity;
            updates++;
        }
        @Override protected void setVelocitySetpoint(AngularVelocity velocity, int slot) {
            requestedVelocity = velocity.in(RadiansPerSecond);
            this.slot = slot;
        }
        @Override protected void setPositionSetpoint(Angle position, int slot) {
            requestedPosition = position.in(Radians);
            this.slot = slot;
        }
    }

    @Test void moduleConvertsSpeedAndUsesConfiguredSlots() {
        var drive = new FakeMotor();
        var turn = new FakeMotor();
        var module = new SwerveModule(drive, turn, Degrees.of(101.8), RADIUS);
        // Sensor reads 101.8-degree mounting offset + 30-degree physical steering angle.
        turn.position = Math.toRadians(131.8);
        module.updateInputs();
        module.setVelocity(new SwerveModuleVelocity(MetersPerSecond.of(2), Rotation2d.fromDegrees(30)));
        assertEquals(40, drive.requestedVelocity, 1e-9);
        assertEquals(Math.toRadians(131.8), turn.requestedPosition, 1e-9);
        assertEquals(0, drive.slot);
        assertEquals(0, turn.slot);
    }

    @Test void odometryRefreshesInputsAndConvertsFieldVelocitiesAfterPoseReset() {
        FakeMotor[] drives = new FakeMotor[4];
        SwerveModule[] modules = new SwerveModule[4];
        for (int i = 0; i < 4; i++) {
            drives[i] = new FakeMotor();
            modules[i] = new SwerveModule(drives[i], new FakeMotor(), Degrees.zero(), RADIUS);
        }
        var swerve = new SwerveDrive(modules, () -> Degrees.zero(), new Translation2d[] {
            new Translation2d(0.3, 0.2), new Translation2d(0.3, -0.2),
            new Translation2d(-0.3, 0.2), new Translation2d(-0.3, -0.2)
        });
        for (var drive : drives) {
            // With a 0.05 m wheel radius, 20 rad is 1 m and 40 rad/s is 2 m/s.
            drive.position = 20;
            drive.velocity = 40;
        }
        swerve.updateTelemetry();
        assertEquals(1, swerve.getPose().getX(), 1e-9);
        assertEquals(2, swerve.getFieldRelativeVelocity().vx, 1e-9);
        assertEquals(2, drives[0].updates);

        swerve.resetOdometry(new Pose2d(1, 0, Rotation2d.fromDegrees(90)));
        assertEquals(0, swerve.getFieldRelativeVelocity().vx, 1e-9);
        assertEquals(2, swerve.getFieldRelativeVelocity().vy, 1e-9);
        swerve.setFieldRelativeChassisVelocities(new ChassisVelocities(1, 0, 0));
        for (var module : modules) {
            assertEquals(-Math.PI / 2, ((FakeMotor) module.mTurnSpark).requestedPosition, 1e-9);
            // Wheels are still perpendicular to the requested direction.
            assertEquals(0, ((FakeMotor) module.mDriveFx).requestedVelocity, 1e-9);
        }
    }

    @Test void reversingDirectionReversesWheelInsteadOfTurningHalfRevolution() {
        var drive = new FakeMotor();
        var turn = new FakeMotor();
        var module = new SwerveModule(drive, turn, Degrees.of(20), RADIUS);
        turn.position = Math.toRadians(20);
        module.updateInputs();
        module.setVelocity(new SwerveModuleVelocity(MetersPerSecond.of(2), Rotation2d.k180deg));
        assertEquals(-40, drive.requestedVelocity, 1e-9);
        assertEquals(Math.toRadians(20), turn.requestedPosition, 1e-9);
    }

    @Test void combinedTranslationAndRotationPreservesRatiosAndLimitsWheelSpeed() {
        Translation2d[] locations = {
            new Translation2d(0.3, 0.2), new Translation2d(0.3, -0.2),
            new Translation2d(-0.3, 0.2), new Translation2d(-0.3, -0.2)
        };
        var request = new ChassisVelocities(5, 3, 10);
        var raw = new SwerveDriveKinematics(locations).toSwerveModuleVelocities(request);
        double max = java.util.Arrays.stream(raw).mapToDouble(v -> v.velocity).max().orElseThrow();
        var modules = new SwerveModule[4];
        for (int i = 0; i < 4; i++) {
            var turn = new FakeMotor();
            // Begin aligned so cosine scaling does not hide the desaturation ratio under test.
            turn.position = raw[i].angle.getRadians();
            modules[i] = new SwerveModule(new FakeMotor(), turn, Degrees.zero(), RADIUS);
        }
        var swerve = new SwerveDrive(modules, () -> Degrees.zero(), locations);
        swerve.setRobotRelativeChassisVelocities(request);
        double limit = frc.robot.subsystems.drive.DriveConstants.kMaxVelocity.in(MetersPerSecond);
        for (int i = 0; i < 4; i++) {
            double speed = ((FakeMotor) modules[i].mDriveFx).requestedVelocity * RADIUS;
            assertEquals(raw[i].velocity * limit / max, speed, 1e-9);
            assertTrue(Math.abs(speed) <= limit + 1e-9);
        }
    }
}
