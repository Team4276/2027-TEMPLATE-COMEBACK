package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import frc.robot.controlboard.ControlBoardConstants;

/** Guards driver speed limiting and the axis/order convention used by module geometry. */
class DriveConstantsTest {
    @Test void forwardAndReverseHaveNoSidewaysOrRotationRequest() {
        var speeds = DriveConstants.getRequestedSpeeds(-1, 0);
        double scale = ControlBoardConstants.kDriveSpeedScale;
        assertEquals(DriveConstants.kMaxVelocity.baseUnitMagnitude() * scale,
                speeds.vx, 1e-9);
        assertEquals(0, speeds.vy, 1e-9);
        assertEquals(0, speeds.omega, 1e-9);
        var reverse = DriveConstants.getRequestedSpeeds(1, 0);
        assertEquals(-speeds.vx, reverse.vx, 1e-9);
        assertEquals(0, reverse.vy, 1e-9);
        assertEquals(0, reverse.omega, 1e-9);
        var centered = DriveConstants.getRequestedSpeeds(0, 0);
        assertEquals(0, Math.hypot(centered.vx, centered.vy), 1e-9);
        assertEquals(0, centered.omega, 1e-9);
    }

    @Test void rotationIsIndependentAndRequestsRemainSpeedLimited() {
        var speeds = DriveConstants.getRequestedSpeeds(0, 1);
        assertEquals(0, speeds.vx, 1e-9);
        assertEquals(0, speeds.vy, 1e-9);
        assertEquals(-DriveConstants.kMaxOmega.baseUnitMagnitude() * ControlBoardConstants.kDriveSpeedScale,
                speeds.omega, 1e-9);
        var clamped = DriveConstants.getRequestedSpeeds(-2, 2);
        assertEquals(DriveConstants.kMaxVelocity.baseUnitMagnitude() * ControlBoardConstants.kDriveSpeedScale,
                clamped.vx, 1e-9);
        assertEquals(speeds.omega, clamped.omega, 1e-9);
    }

    @Test void frontLeftMatchesMeasuredForwardAndLeftDistances() {
        assertEquals(0.34925, DriveConstants.kModuleTranslations[0].getX(), 1e-9);
        assertEquals(0.24765, DriveConstants.kModuleTranslations[0].getY(), 1e-9);
    }
}
