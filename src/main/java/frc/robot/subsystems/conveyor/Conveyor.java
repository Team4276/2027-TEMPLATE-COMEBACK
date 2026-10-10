package frc.robot.subsystems.conveyor;

//import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;


public class Conveyor extends MotorSubsystem<MotorIO> {
    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    public static final Setpoint CONVEY = Setpoint.withVelocitySetpoint(ConveyorConstants.kConveyVelocity, 0);
    public static final Setpoint INVERSCONVEY = Setpoint.withVelocitySetpoint(ConveyorConstants.kInversConveyVelocity, 0);

    public static final Conveyor mInstance = new Conveyor();

    public Conveyor() {
        super(ConveyorConstants.getMotorIO(), "Conveyor");
    }
}
