package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import frc.robot.controlboard.ControlBoardConstants;

/** Guards driver speed limiting and the axis/order convention used by module geometry. */
class DriveConstantsTest {
    @Test void diagonalInputIsClampedBeforeSquaring() {
        var speeds = DriveConstants.getRequestedSpeeds(1, 1, 0);
        double scale = ControlBoardConstants.kDriveBringupMode ? 0.2 : 1.0;
        assertEquals(DriveConstants.kMaxVelocity.baseUnitMagnitude() * scale,
                Math.hypot(speeds.vx, speeds.vy), 1e-9);
        var centered = DriveConstants.getRequestedSpeeds(0, 0, 0);
        assertEquals(0, Math.hypot(centered.vx, centered.vy), 1e-9);
        assertEquals(0, centered.omega, 1e-9);
    }

    @Test void frontLeftMatchesMeasuredForwardAndLeftDistances() {
        assertEquals(0.34925, DriveConstants.kModuleTranslations[0].getX(), 1e-9);
        assertEquals(0.24765, DriveConstants.kModuleTranslations[0].getY(), 1e-9);
    }
}
