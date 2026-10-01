package frc.lib.hid;

/** Immutable raw stick vector; axis remapping into robot coordinates belongs to the drive request. */
public class JoystickOutput {
    public final double x;
    public final double y;

    public JoystickOutput(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public JoystickOutput() {
        this.x = 0.0;
        this.y = 0.0;
    }

    /** Squares each axis with its sign preserved; unlike radial squaring, this can change direction. */
    public JoystickOutput sq() {
        return new JoystickOutput(
                Math.copySign(x * x, x),
                Math.copySign(y * y, y));
    }
}
