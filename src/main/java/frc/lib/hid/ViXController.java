package frc.lib.hid;

import org.wpilib.command2.button.CommandNiDsXboxController;

public class ViXController extends CommandNiDsXboxController implements JoystickOutputController {
    private double JOYSTICK_DEADBAND = 0.1;
    private double TRIGGER_DEADBAND = 0.25;

    public ViXController(int port) {
        super(port);
    }

    public ViXController(int port, double deadband) {
        super(port);
        this.JOYSTICK_DEADBAND = deadband;
    }

    public void setDeadband(double deadband) {
        this.JOYSTICK_DEADBAND = deadband;
    }

    @Override
    public JoystickOutput getRightWithDeadband() {
        return Math.hypot(getRightX(), getRightY()) < JOYSTICK_DEADBAND
                ? new JoystickOutput()
                : getRight();
    }

    @Override
    public JoystickOutput getRight() {
        return new JoystickOutput(getRightX(), getRightY());
    }

    @Override
    public JoystickOutput getLeftWithDeadband() {
        return Math.hypot(getLeftX(), getLeftY()) < JOYSTICK_DEADBAND
                ? new JoystickOutput()
                : getLeft();
    }

    @Override
    public JoystickOutput getLeft() {
        return new JoystickOutput(getLeftX(), getLeftY());
    }

    public boolean getLT() {
        return getLeftTriggerAxis() > TRIGGER_DEADBAND;
    }

    public boolean getRT() {
        return getRightTriggerAxis() > TRIGGER_DEADBAND;
    }

    // public Command rumbleCommand(RumbleType type, double value, double duration) {
    //     return rumbleCommand(type, value, duration, 1);
    // }

    // public Command rumbleCommand(RumbleType type, double value, double duration, int times) {
    //     var command = new SequentialCommandGroup();

    //     // for (int i = 0; i < times; i++) {
    //     //     command.addCommands(
    //     //             Commands.startEnd(() -> setR(type, value), () -> setRumble(type, 0.0))
    //     //                     .withTimeout(duration)
    //     //                     .andThen(Commands.waitSeconds(0.1)));
    //     // }

    //     return command;
    // }
}
