package frc.lib.hid;

import org.wpilib.command2.button.CommandXboxController;

public class ViXController extends CommandXboxController implements JoystickOutputController {
    private double JOYSTICK_DEADBAND = 0.1;

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
    
}
