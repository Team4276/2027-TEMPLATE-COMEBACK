package frc.lib.hid;

import org.wpilib.command2.button.CommandXboxController;

/**
 * Xbox layout for the FIRST Driver Station used with SystemCore. The legacy NI layout swaps
 * LT with right-stick X and also uses different bumper/menu button numbers.
 * Independent axis deadbands prevent an unused stick axis from enabling drift on another.
 */
public class ViXController extends CommandXboxController implements JoystickOutputController {
    // These thresholds apply to getLT/getRT, not the inherited command Trigger factories.
    private double TRIGGER_DEADBAND = 0.25;

    public ViXController(int port) {
        this(port, 0.1);
    }

    public ViXController(int port, double deadband) {
        super(port);
        setDeadband(deadband);
    }

    public void setDeadband(double deadband) {
        getController().setLeftXDeadband(deadband);
        getController().setLeftYDeadband(deadband);
        getController().setRightXDeadband(deadband);
        getController().setRightYDeadband(deadband);
    }

    @Override
    public JoystickOutput getRightWithDeadband() {
        return getRight();
    }

    @Override
    public JoystickOutput getRight() {
        return new JoystickOutput(getRightX(), getRightY());
    }

    @Override
    public JoystickOutput getLeftWithDeadband() {
        return getLeft();
    }

    @Override
    public JoystickOutput getLeft() {
        return new JoystickOutput(getLeftX(), getLeftY());
    }

    public boolean getLT() {
        return getLeftTrigger() > TRIGGER_DEADBAND;
    }

    public boolean getRT() {
        return getRightTrigger() > TRIGGER_DEADBAND;
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
