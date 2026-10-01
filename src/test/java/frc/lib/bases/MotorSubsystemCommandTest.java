package frc.lib.bases;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.button.Trigger;
import org.wpilib.event.EventLoop;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.units.measure.Voltage;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;

class MotorSubsystemCommandTest {
    private static class FakeMotor extends MotorIO {
        double voltage;
        FakeMotor() { super(Rotations, Seconds); }
        @Override public void updateInputs() {}
        @Override protected void setVoltageSetpoint(Voltage value) { voltage = value.in(Volts); }
        @Override protected void setNeutralSetpoint() { voltage = 0; }
    }

    private MotorSubsystem<FakeMotor> mechanism;
    private FakeMotor motor;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final EventLoop buttons = new EventLoop();
    private final AtomicBoolean held = new AtomicBoolean();
    private Command command;

    @BeforeEach void setup() {
        assertTrue(HAL.initialize());
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        motor = new FakeMotor();
        mechanism = new MotorSubsystem<>(motor, "HeldMechanismTest");
        command = mechanism.holdSetpointCommand(Setpoint.withVoltageSetpoint(Volts.of(6)));
        new Trigger(buttons, held::get).whileTrue(command);
    }

    @AfterEach void cleanup() {
        scheduler.cancelAll();
        scheduler.unregisterSubsystem(mechanism);
        buttons.clear();
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    private void tick() {
        buttons.poll();
        scheduler.run();
    }

    @Test void noButtonOnEnableStaysStoppedAndReleaseStopsVoltage() {
        tick();
        assertEquals(0, motor.voltage);
        held.set(true);
        tick();
        tick();
        assertTrue(command.isScheduled());
        assertEquals(6, motor.voltage);
        held.set(false);
        tick();
        assertFalse(command.isScheduled());
        assertEquals(0, motor.voltage);
        assertSame(Setpoint.NEUTRAL, motor.getSetpoint());
    }

    @Test void oppositeRequestInterruptsAndReleasingItStops() {
        held.set(true);
        tick();
        var reverse = mechanism.holdSetpointCommand(Setpoint.withVoltageSetpoint(Volts.of(-6)));
        scheduler.schedule(reverse);
        scheduler.run();
        assertFalse(command.isScheduled());
        assertEquals(-6, motor.voltage);
        reverse.cancel();
        assertEquals(0, motor.voltage);
    }

    @Test void disablingStopsAHeldCommandAndDoesNotRestartItOnEnable() {
        held.set(true);
        tick();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        tick();
        assertFalse(command.isScheduled());
        assertEquals(0, motor.voltage);
        held.set(false);
        tick();
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        tick();
        assertEquals(0, motor.voltage);
    }
}
