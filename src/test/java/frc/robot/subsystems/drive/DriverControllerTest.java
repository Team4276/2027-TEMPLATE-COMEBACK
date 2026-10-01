package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.event.EventLoop;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.XboxControllerSim;
import org.wpilib.units.measure.Voltage;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.controlboard.ControlBoard;

/** Exercises FIRST DS packets, not assumed logical button values, to catch layout mismatches. */
class DriverControllerTest {
    private static class FakeMotor extends MotorIO {
        double voltage;
        FakeMotor() { super(Rotations, Seconds); }
        @Override public void updateInputs() {}
        @Override protected void setVoltageSetpoint(Voltage value) { voltage = value.in(Volts); }
        @Override protected void setNeutralSetpoint() { voltage = 0; }
    }

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final EventLoop buttons = new EventLoop();
    private XboxControllerSim controllerSim;
    private MotorSubsystem<FakeMotor> deploy;
    private MotorSubsystem<FakeMotor> rollers;

    @BeforeEach void setup() {
        assertTrue(HAL.initialize());
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
        DriverStationSim.setEnabled(true);
        controllerSim = new XboxControllerSim(0);
        deploy = new MotorSubsystem<>(new FakeMotor(), "ControllerTestDeploy");
        rollers = new MotorSubsystem<>(new FakeMotor(), "ControllerTestRollers");
        ControlBoard.mDriver.leftTrigger(0.5, buttons)
                .and(DriverStationBackend::isTeleopEnabled)
                .whileTrue(deploy.holdSetpointCommand(Setpoint.withVoltageSetpoint(Volts.of(6)))
                        .alongWith(rollers.holdSetpointCommand(Setpoint.withVoltageSetpoint(Volts.of(12)))));
        tick();
        assertTrue(DriverStationBackend.isTeleopEnabled());
    }

    private void tick() {
        DriverStationSim.notifyNewData();
        buttons.poll();
        scheduler.run();
    }

    @AfterEach void cleanup() {
        scheduler.cancelAll();
        scheduler.unregisterSubsystem(deploy, rollers);
        buttons.clear();
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test void leftTriggerCannotDriveAndReleaseStopsBothIntakeMotors() {
        assertEquals(0, deploy.getIO().voltage);
        assertEquals(0, rollers.getIO().voltage);
        controllerSim.setLeftTrigger(1);
        tick();
        var speeds = DriveConstants.kTeleopRequestUpdater.get();
        assertEquals(0, speeds.vx, 1e-9);
        assertEquals(0, speeds.vy, 1e-9);
        assertEquals(0, speeds.omega, 1e-9);
        assertEquals(6, deploy.getIO().voltage);
        assertEquals(12, rollers.getIO().voltage);

        // Releasing LT must stop intake even when right-stick X remains displaced.
        controllerSim.setRightX(1);
        controllerSim.setLeftTrigger(0);
        tick();
        tick();
        assertEquals(0, deploy.getIO().voltage);
        assertEquals(0, rollers.getIO().voltage);
        assertTrue(DriveConstants.kTeleopRequestUpdater.get().omega < 0);
    }

    @Test void unusedStickAxesAndRightTriggerCannotMoveTheRobotOrDeployIntake() {
        controllerSim.setLeftX(1);
        controllerSim.setRightY(1);
        controllerSim.setRightTrigger(1);
        // Noise on the used axes remains inside their own deadbands.
        controllerSim.setLeftY(0.04);
        controllerSim.setRightX(0.04);
        tick();
        var speeds = DriveConstants.kTeleopRequestUpdater.get();
        assertEquals(0, speeds.vx, 1e-9);
        assertEquals(0, speeds.vy, 1e-9);
        assertEquals(0, speeds.omega, 1e-9);
        assertEquals(0, deploy.getIO().voltage);
        assertEquals(0, rollers.getIO().voltage);
    }

    @Test void firstDsBumpersAndMenuMatchTheirPhysicalButtons() {
        var left = ControlBoard.mDriver.leftBumper(buttons);
        var right = ControlBoard.mDriver.rightBumper(buttons);
        var menu = ControlBoard.mDriver.menu(buttons);
        var view = ControlBoard.mDriver.view(buttons);
        controllerSim.setLeftBumperButton(true);
        tick();
        assertTrue(left.getAsBoolean());
        assertFalse(right.getAsBoolean());
        assertFalse(menu.getAsBoolean());
        controllerSim.setLeftBumperButton(false);
        controllerSim.setRightBumperButton(true);
        tick();
        assertFalse(left.getAsBoolean());
        assertTrue(right.getAsBoolean());
        controllerSim.setRightBumperButton(false);
        controllerSim.setMenuButton(true);
        tick();
        assertFalse(right.getAsBoolean());
        assertTrue(menu.getAsBoolean());
        assertFalse(view.getAsBoolean());
    }
}
