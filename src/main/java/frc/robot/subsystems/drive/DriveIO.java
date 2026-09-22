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

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        Pose2d pose = Pose2d.ZERO;
        Angle gyroAngle = Units.Degrees.of(0);
        ChassisVelocities fieldRelativeSpeed = new ChassisVelocities();
        ChassisVelocities robotRelativeSpeed = new ChassisVelocities();

        SwerveModulePosition[] modulesPositions = new SwerveModulePosition[] {};
        SwerveModuleVelocity[] moduleStates = new SwerveModuleVelocity[] {};

        ModuleInput module0Inputs = new ModuleInput(true, 0, 0, 0, 0, 0, 0, true, 0, 0, 0, 0, 0, 0);
        ModuleInput module1Inputs = new ModuleInput(true, 0, 0, 0, 0, 0, 0, true, 0, 0, 0, 0, 0, 0);
        ModuleInput module2Inputs = new ModuleInput(true, 0, 0, 0, 0, 0, 0, true, 0, 0, 0, 0, 0, 0);
        ModuleInput module3Inputs = new ModuleInput(true, 0, 0, 0, 0, 0, 0, true, 0, 0, 0, 0, 0, 0);

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