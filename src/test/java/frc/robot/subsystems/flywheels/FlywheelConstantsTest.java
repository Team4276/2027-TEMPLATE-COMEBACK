package frc.robot.subsystems.flywheels;

import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import org.junit.jupiter.api.Test;

class FlywheelConstantsTest {
    @Test void shooterUsesDirectVelocityWithFeedforwardInRotationsPerSecond() {
        var config = FlywheelConstants.getIOConfig();
        var request = assertInstanceOf(VelocityVoltage.class,
                config.requestGetter.getVelocityRequest(FlywheelConstants.kShubVelocity, 0));
        assertEquals(2500.0 / 60.0, request.Velocity, 1e-9);
        assertEquals(0, request.Slot);
        // The declared 6000 RPM / 12 V motor model calls for roughly 5 V at 2500 RPM.
        assertEquals(5.0, request.Velocity * config.mainConfig.Slot0.kV, 1e-9);
    }
}
