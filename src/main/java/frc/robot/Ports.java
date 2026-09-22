package frc.robot;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;

public enum Ports {
    /* Drive */
    FRONT_LEFT_DRIVE(1, CANPort.CAN_S1),
    FRONT_LEFT_TURN(2, CANPort.CAN_S1),
    FRONT_RIGHT_DRIVE(3, CANPort.CAN_S1),
    FRONT_RIGHT_TURN(4, CANPort.CAN_S1),
    BACK_LEFT_DRIVE(5, CANPort.CAN_S1),
    BACK_LEFT_TURN(6, CANPort.CAN_S1),
    BACK_RIGHT_DRIVE(7, CANPort.CAN_S1),
    BACK_RIGHT_TURN(8, CANPort.CAN_S1),
    PIGEON(9, CANPort.CAN_S1),

    /* Intake */
    INTAKE_DEPLOY(10, CANPort.CAN_S2),
    INTAKE_ROLLERS(11, CANPort.CAN_S2),

    /* Feeder */
    FEEDER(12, CANPort.CAN_S2),
    HOPPER(13, CANPort.CAN_S2),

    /* Shooter */
    FLYWHEEL_LEFT(14, CANPort.CAN_S2),
    FLYWHEEL_RIGHT(15, CANPort.CAN_S2),

    ;

    public final int id;
    public final CANBus bus;
    public final CANPort canPort;

    public CANBus toCANBus(CANPort canPort) {
        return switch (canPort) {
            case CAN_S0 -> RobotConstants.S0;
            case CAN_S1 -> RobotConstants.S1;
            case CAN_S2 -> RobotConstants.S2;
            case CAN_S3 -> RobotConstants.S3;
            case CAN_S4 -> RobotConstants.S4;
            default -> RobotConstants.S0;
        };
    }
    
    private Ports(int id, CANPort canPort) {
        this.id = id;
        this.canPort = canPort;
        this.bus = toCANBus(canPort);
    }
}
