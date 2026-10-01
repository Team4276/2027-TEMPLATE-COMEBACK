package frc.robot.subsystems.drive;

// import static org.wpilib.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
// import org.wpilib.units.measure.AngularVelocity;
// import org.wpilib.units.measure.Current;
// import org.wpilib.units.measure.Temperature;
// import org.wpilib.units.measure.Voltage;
// import frc.lib.util.vision.VisionEstimate;

/**
 * Boundary between command logic and drivetrain implementation. Empty defaults allow a passive
 * implementation; hardware updates and simulation behavior must be supplied by an adapter.
 */
public interface DriveIO {
    // AutoLog generates DriveIOInputsAutoLogged; edit this schema, not generated build output.
    @AutoLog
    public static class DriveIOInputs {
        Pose2d pose = Pose2d.ZERO;
        boolean[] driveConfigFailed = new boolean[4];
        boolean[] turnConfigFailed = new boolean[4];
        Angle gyroAngle = Units.Degrees.of(0);
        ChassisVelocities fieldRelativeSpeed = new ChassisVelocities();
        ChassisVelocities robotRelativeSpeed = new ChassisVelocities();

        SwerveModulePosition[] modulesPositions = new SwerveModulePosition[] {};
        SwerveModuleVelocity[] moduleStates = new SwerveModuleVelocity[] {};
        // All diagnostic arrays use FL, FR, BL, BR order. Targets precede cosine scaling;
        // applied target speed shows how much steering error reduces each wheel's drive request.
        SwerveModuleVelocity[] moduleTargets = new SwerveModuleVelocity[] {};
        double[] absoluteEncoderDegrees = new double[4];
        double[] steeringErrorDegrees = new double[4];
        double[] appliedTargetMetersPerSecond = new double[4];

        ModuleInput module0Inputs = new ModuleInput(false, 0, 0, 0, 0, 0, 0, false, 0, 0, 0, 0, 0, 0);
        ModuleInput module1Inputs = new ModuleInput(false, 0, 0, 0, 0, 0, 0, false, 0, 0, 0, 0, 0, 0);
        ModuleInput module2Inputs = new ModuleInput(false, 0, 0, 0, 0, 0, 0, false, 0, 0, 0, 0, 0, 0);
        ModuleInput module3Inputs = new ModuleInput(false, 0, 0, 0, 0, 0, 0, false, 0, 0, 0, 0, 0, 0);

        // ModuleInput module0Inputs = new ModuleInput(
        //     true,
        //     Angle.ofBaseUnits(0, Rotations),
        //     AngularVelocity.ofBaseUnits(0, RotationsPerSecond),
        //     Voltage.ofBaseUnits(0, Volts),
        //     Current.ofBaseUnits(0, Amps),
        //     Current.ofBaseUnits(0, Amps),
        //     Temperature.ofBaseUnits(0, Celsius),

        //     true,
        //     Angle.ofBaseUnits(0, Radians),
        //     AngularVelocity.ofBaseUnits(0, RadiansPerSecond),
        //     Voltage.ofBaseUnits(0, Volts),
        //     Current.ofBaseUnits(0, Amps),
        //     Current.ofBaseUnits(0, Amps),
        //     Temperature.ofBaseUnits(0, Celsius)
        // );
        // ModuleInput module1Inputs = new ModuleInput(
        //     true,
        //     Angle.ofBaseUnits(0, Rotations),
        //     AngularVelocity.ofBaseUnits(0, RotationsPerSecond),
        //     Voltage.ofBaseUnits(0, Volts),
        //     Current.ofBaseUnits(0, Amps),
        //     Current.ofBaseUnits(0, Amps),
        //     Temperature.ofBaseUnits(0, Celsius),

        //     true,
        //     Angle.ofBaseUnits(0, Radians),
        //     AngularVelocity.ofBaseUnits(0, RadiansPerSecond),
        //     Voltage.ofBaseUnits(0, Volts),
        //     Current.ofBaseUnits(0, Amps),
        //     Current.ofBaseUnits(0, Amps),
        //     Temperature.ofBaseUnits(0, Celsius)
        // );
        // ModuleInput module2Inputs = new ModuleInput(
        //     true,
        //     Angle.ofBaseUnits(0, Rotations),
        //     AngularVelocity.ofBaseUnits(0, RotationsPerSecond),
        //     Voltage.ofBaseUnits(0, Volts),
        //     Current.ofBaseUnits(0, Amps),
        //     Current.ofBaseUnits(0, Amps),
        //     Temperature.ofBaseUnits(0, Celsius),

        //     true,
        //     Angle.ofBaseUnits(0, Radians),
        //     AngularVelocity.ofBaseUnits(0, RadiansPerSecond),
        //     Voltage.ofBaseUnits(0, Volts),
        //     Current.ofBaseUnits(0, Amps),
        //     Current.ofBaseUnits(0, Amps),
        //     Temperature.ofBaseUnits(0, Celsius)
        // );
        // ModuleInput module3Inputs = new ModuleInput(
        //     true,
        //     Angle.ofBaseUnits(0, Rotations),
        //     AngularVelocity.ofBaseUnits(0, RotationsPerSecond),
        //     Voltage.ofBaseUnits(0, Volts),
        //     Current.ofBaseUnits(0, Amps),
        //     Current.ofBaseUnits(0, Amps),
        //     Temperature.ofBaseUnits(0, Celsius),

        //     true,
        //     Angle.ofBaseUnits(0, Radians),
        //     AngularVelocity.ofBaseUnits(0, RadiansPerSecond),
        //     Voltage.ofBaseUnits(0, Volts),
        //     Current.ofBaseUnits(0, Amps),
        //     Current.ofBaseUnits(0, Amps),
        //     Temperature.ofBaseUnits(0, Celsius)
        // );
    }

    public default void updateInputs(DriveIOInputs inputs) {
    }

    public default void updateSim() {
    }

    public default void resetPose(Pose2d pose) {
    }

    /** Requests field-relative translation in m/s and rotation in rad/s. */
    public default void drive(ChassisVelocities speeds) {
    }

    // public default void addVisionMeasurement(VisionEstimate estimate) {
    // }

    // public static record ModuleInput(
    //         boolean driveConnected,
    //         Angle driveRotorPosition,
    //         AngularVelocity driveRotorVelocity,
    //         Voltage driveVoltage,
    //         Current driveSupplyCurrent,
    //         Current driveStatorCurrent,
    //         Temperature driveTemp,

    //         boolean turnConnected,
    //         Angle turnEncoderPosition,
    //         AngularVelocity turnEncoderVelocity,
    //         Voltage turnVoltage,
    //         Current turnSupplyCurrent,
    //         Current turnStatorCurrent,
    //         Temperature turnTemp) {
    // }
    
    /**
     * Flattened motor measurements for logging. Angles and angular speeds are base units
     * (radians and rad/s), even where the legacy field name says "rotor": the adapters may
     * already account for gearing. Temperatures follow the motor adapter's representation.
     */
    public static record ModuleInput(
            boolean driveConnected,
            double driveRotorPosition,
            double driveRotorVelocity,
            double driveVoltage,
            double driveSupplyCurrent,
            double driveStatorCurrent,
            double driveTemp,

            boolean turnConnected,
            double turnEncoderPosition,
            double turnEncoderVelocity,
            double turnVoltage,
            double turnSupplyCurrent,
            double turnStatorCurrent,
            double turnTemp) {
    }
}
