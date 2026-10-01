package frc.lib.bases;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.units.measure.AngularVelocity;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;

class MotorSubsystemTest {
    private static class FakeMotor extends MotorIO {
        // Represent a controller with a non-neutral output before this program takes ownership.
        double requestedRpm = 2500;
        int neutralRequests;

        FakeMotor() { super(Rotations, Minutes); }
        @Override public void updateInputs() {}
        @Override protected void setNeutralSetpoint() {
            requestedRpm = 0;
            neutralRequests++;
        }
        @Override protected void setVelocitySetpoint(AngularVelocity velocity, int slot) {
            requestedRpm = velocity.in(RPM);
        }
    }

    private final FakeMotor motor = new FakeMotor();
    private final MotorSubsystem<FakeMotor> mechanism = new MotorSubsystem<>(motor, "StartupTest");
    private final Setpoint shoot = Setpoint.withVelocitySetpoint(RPM.of(2500), 0);

    @AfterEach void unregisterSubsystem() {
        CommandScheduler.getInstance().unregisterSubsystem(mechanism);
    }

    @Test void constructionActuallySendsNeutralBeforeAnyButtonCommand() {
        assertEquals(1, motor.neutralRequests);
        assertEquals(0, motor.requestedRpm);
        assertSame(Setpoint.NEUTRAL, motor.getSetpoint());
        // Enabling without an intervening command must still send neutral, not a velocity request.
        mechanism.enable();
        assertEquals(0, motor.requestedRpm);
    }

    @Test void stoppingClearsAShotSoItCannotResumeOnEnable() {
        mechanism.applySetpoint(shoot);
        assertEquals(2500, motor.requestedRpm);
        mechanism.stop();
        assertEquals(0, motor.requestedRpm);
        assertSame(Setpoint.NEUTRAL, motor.getSetpoint());
        mechanism.enable();
        assertEquals(0, motor.requestedRpm);

        // Explicit commands still work after the lifecycle reset in normal operation.
        mechanism.applySetpoint(shoot);
        assertEquals(2500, motor.requestedRpm);
    }

    @Test void bringupDisableBlocksShotRequestsAndStopDiscardsQueuedShot() {
        mechanism.stop();
        mechanism.disable();
        mechanism.applySetpoint(shoot);
        assertFalse(motor.getEnabled());
        assertEquals(0, motor.requestedRpm);

        mechanism.stop();
        mechanism.enable();
        assertEquals(0, motor.requestedRpm);
        assertSame(Setpoint.NEUTRAL, motor.getSetpoint());
    }
}
