package frc.robot.subsystems.drive.scuffed;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.*;
import org.wpilib.math.kinematics.*;
import org.wpilib.units.measure.*;
import frc.lib.io.MotorIO;

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
            assertEquals(20, ((FakeMotor) module.mDriveFx).requestedVelocity, 1e-9);
        }
    }
}
