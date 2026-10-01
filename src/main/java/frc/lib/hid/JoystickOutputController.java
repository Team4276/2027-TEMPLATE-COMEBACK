package frc.lib.hid;

/** Common stick access for gamepads/dual joysticks; unimplemented axes default to centered input. */
public interface JoystickOutputController {

    default JoystickOutput getRight() {
        return new JoystickOutput();
    }

    default JoystickOutput getRightWithDeadband() {
        return new JoystickOutput();
    }

    default JoystickOutput getLeft() {
        return new JoystickOutput();
    }

    default JoystickOutput getLeftWithDeadband() {
        return new JoystickOutput();
    }
}
