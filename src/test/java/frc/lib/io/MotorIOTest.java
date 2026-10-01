package frc.lib.io;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import org.junit.jupiter.api.Test;
import frc.lib.bases.ServoMotorSubsystem;

/** Regression checks for the IO contract and servo target comparisons without real CAN devices. */
class MotorIOTest {
    private static class FakeMotor extends MotorIO {
        FakeMotor(int followers) { super(Rotations, Seconds, followers); }
        @Override public void updateInputs() {}
    }

    @Test void allocatesEveryTelemetryArrayForMainAndFollowers() {
        for (int followers : new int[] {0, 1, 3}) {
            var motor = new FakeMotor(followers);
            for (double[] values : new double[][] {
                motor.inputs.position, motor.inputs.velocity, motor.inputs.statorCurrent,
                motor.inputs.supplyCurrent, motor.inputs.motorVoltage,
                motor.inputs.motorTemperature, motor.inputs.acceleration
            }) {
                assertEquals(followers + 1, values.length);
                values[followers] = 42;
                assertEquals(42, values[followers]);
            }
        }
    }

    @Test void servoChecksTargetInsteadOfComparingPositionWithItself() {
        var motor = new FakeMotor(0);
        var servo = new ServoMotorSubsystem<>(motor, "TestServo", Degrees.of(1));
        try {
            servo.applySetpoint(MotorIO.Setpoint.withPositionSetpoint(Degrees.of(90)));
            assertFalse(servo.nearPositionSetpoint());
            motor.inputs.position[0] = Degrees.of(89.5).baseUnitMagnitude();
            assertTrue(servo.nearPositionSetpoint());
            servo.applySetpoint(MotorIO.Setpoint.withNeutralSetpoint());
            assertFalse(servo.nearPositionSetpoint());
        } finally {
            org.wpilib.command2.CommandScheduler.getInstance().unregisterSubsystem(servo);
        }
    }
}
