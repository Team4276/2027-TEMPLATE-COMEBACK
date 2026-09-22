package frc.robot;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;

public class RobotConstants {
    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY,
    }

    public static Mode getMode() {
        return mode;
    }

    public static enum RobotType {
        COMPBOT,
        SIMBOT
    }

    public static Mode mode = Mode.REAL;

    public static RobotType getType() {
        return switch (mode) {
            case REAL -> RobotType.COMPBOT;
            case REPLAY -> RobotType.COMPBOT;
            case SIM -> RobotType.SIMBOT;
        };
    }

    public static final CANBus canivore1 = new CANBus("canivore1");
    public static final CANBus rio = new CANBus();
    public static final CANBus S0 = new CANBus(CANPort.CAN_S0);
    public static final CANBus S1 = new CANBus(CANPort.CAN_S1);
    public static final CANBus S2 = new CANBus(CANPort.CAN_S2);
    public static final CANBus S3 = new CANBus(CANPort.CAN_S3);
    public static final CANBus S4 = new CANBus(CANPort.CAN_S4);

    public static final boolean isTuning = true;

    public static final boolean simulateVision = false;
}
