package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.LoggedTunablePID;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.ControlBoardConstants;

public class DriveConstants {
    public static final LinearVelocity kMaxVelocity = MetersPerSecond.of(5.623);
    public static final AngularVelocity kMaxOmega = RadiansPerSecond.of(13.154);

    public static final double trackWidth = Units.inchesToMeters(19.5);
    public static final double wheelBase = Units.inchesToMeters(27.5);
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };
    
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.47);
    public static final double drivingMotorPinionTeeth = 14.0;
    public static final double driveMotorReduction = (45.0 * 22.0) / (drivingMotorPinionTeeth * 15.0);

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 50.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 50.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 50.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = (12.0 / 100.0);
        config.Slot0.kA = 0.0;

        config.Feedback.SensorToMechanismRatio = driveMotorReduction;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return config;
    }

    public static MotorIOTalonFXConfig getDriveIOConfig(Ports port) {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.unit = org.wpilib.units.Units.Rotations;
        config.time = org.wpilib.units.Units.Minutes;
        config.mainID = port.id;
        config.mainConfig = getFXConfig();
        config.mainBus = port.bus;
        return config;
    }

    public static SparkMaxConfig getSparkConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20)
                .voltageCompensation(12.0);
        config.absoluteEncoder
                .inverted(true)
                .averageDepth(2);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .pid(1.0, 0.0, 0.0);
        config.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / 50))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        return config;
    }

    public static MotorIOSparkMaxConfig getTurnIOConfig(Ports port) {
        MotorIOSparkMaxConfig config = new MotorIOSparkMaxConfig();
        config.unit = org.wpilib.units.Units.Rotations;
        config.time = org.wpilib.units.Units.Minutes;
        config.useAbsoluteEncoder = true;
        config.velocityConversionFactor = 60.0; // For some reason absolute encoder native units is RPM
        config.mainID = port.id;
        config.mainConfig = getSparkConfig();
        config.canPort = port.canPort;
        return config;
    }

    private static ChassisVelocities getRequestedSpeeds(double xInput, double yInput, double rotationInput) {
        double linearMagnitude = Math.hypot(xInput, yInput);

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        Translation2d linearVelocity = Translation2d.ZERO;

        if (linearMagnitude > 1e-6) {
            linearVelocity = new Translation2d(
                    linearMagnitude,
                    new Rotation2d(
                            xInput, yInput))
                    .times(DriveConstants.kMaxVelocity.baseUnitMagnitude());
        }

        // Square rotation value for more precise control
        double omega = Math.copySign(
                rotationInput * rotationInput,
                -rotationInput);

        return new ChassisVelocities(
                linearVelocity.getX(),
                linearVelocity.getY(),
                omega * DriveConstants.kMaxOmega.baseUnitMagnitude())
                .toFieldRelative(AllianceFlipUtil.apply(Rotation2d.k180deg));
    }

    public static final Supplier<ChassisVelocities> kTeleopRequestUpdater = switch (ControlBoardConstants.kInputMode) {
        case CONTROLLER -> () -> {
            return getRequestedSpeeds(
                    ControlBoard.mDriver.getLeftWithDeadband().y,
                    ControlBoard.mDriver.getLeftWithDeadband().x,
                    ControlBoard.mDriver.getRightWithDeadband().x);
        };
        case KEYBOARD -> () -> {
            return getRequestedSpeeds(
                    ControlBoard.mKeyboard0.getRawAxis(1),
                    ControlBoard.mKeyboard0.getRawAxis(0),
                    ControlBoard.mKeyboard1.getRawAxis(0));
        };
        case DEMO -> () -> {
            return getRequestedSpeeds(
                    ControlBoard.mDriver.getLeftWithDeadband().y,
                    ControlBoard.mDriver.getLeftWithDeadband().x,
                    ControlBoard.mDriver.getRightWithDeadband().x);
        };

    };

    public static final LoggedTunablePID kTeleopAutoAlignController = new LoggedTunablePID(
            3.0, 0, 0.1, Units.inchesToMeters(1.0), "Drive/AutoAlign/TeleopTranslation");
    public static final LoggedTunablePID kHeadingAlignController = new LoggedTunablePID(20.0, 0, 0,
            Math.toRadians(1.0),
            "Drive/HeadingAlign");

    public static final LoggedTunablePID kTrajectoryXController = new LoggedTunablePID(5.0, 0, 0,
            Units.inchesToMeters(0.5),
            "Drive/Trajectory/Translation");
    public static final LoggedTunablePID kTrajectoryYController = new LoggedTunablePID(5.0, 0, 0,
            Units.inchesToMeters(0.5),
            "Drive/Trajectory/Translation");
    public static final LoggedTunablePID kTrajectoryThetaController = new LoggedTunablePID(3.0, 0, 0,
            Math.toRadians(1.0),
            "Drive/Trajectory/Rotation");
}