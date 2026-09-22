package frc.lib.util;

import org.wpilib.math.controller.PIDController;
import frc.robot.RobotConstants;

public class LoggedTunablePID extends PIDController {
    public final TunableNumber Kp;
    public final TunableNumber Ki;
    public final TunableNumber Kd;
    public final TunableNumber KTol;

    private final String key;

    public LoggedTunablePID(double kp, double ki, double kd, String key) {
        super(kp, ki, kd);
        this.key = key;
        KTol = new TunableNumber(this.key + "/Tolerance", getErrorTolerance());
        Kd = new TunableNumber(this.key + "/kD", kd);
        Ki = new TunableNumber(this.key + "/kI", ki);
        Kp = new TunableNumber(this.key + "/kP", kp);
    }

    public LoggedTunablePID(double kp, double ki, double kd, double tol, String key) {
        super(kp, ki, kd);
        this.key = key;
        KTol = new TunableNumber(this.key + "/Tolerance", tol);
        Kd = new TunableNumber(this.key + "/kD", kd);
        Ki = new TunableNumber(this.key + "/kI", ki);
        Kp = new TunableNumber(this.key + "/kP", kp);
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        if (RobotConstants.isTuning) {
            setPID(Kp.getAsDouble(), Ki.getAsDouble(), Kd.getAsDouble());
            setTolerance(KTol.getAsDouble());
        }
        return super.calculate(measurement, setpoint);
    }
}
