package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.lib.util.AllianceFlipUtil;
// import frc.lib.util.vision.VisionEstimate;

/** Command-facing drive subsystem. Requests use field coordinates; IO owns hardware and odometry. */
public class Drive extends SubsystemBase {
    public static final Drive mInstance = new Drive();

    // This template currently constructs hardware IO in every mode; a simulated/replay-only
    // drivetrain would need an IO selection here, unlike the mechanism factories.
    private DriveIO io = new DriveIOScuffed();
    private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private Drive() {
    }

    @Override
    public void periodic() {
        // Read before processing so commands see this loop's measurements (or replayed inputs).
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.updateSim();
    }

    public Pose2d getPose() {
        return inputs.pose;
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    public void zeroGyro() {
        // Change the field heading reference while preserving translation; this is a pose reset,
        // not a write to the Pigeon's raw yaw sensor.
        resetPose(
                new Pose2d(
                        getPose().getTranslation(),
                        AllianceFlipUtil.apply(Rotation2d.ZERO)));

    }

    public Command drive(ChassisVelocities speeds) {
        return drive(() -> speeds);
    }

    public Command drive(Supplier<ChassisVelocities> speeds) {
        // Evaluate the supplier each loop for live joystick input and stop when interrupted.
        return Commands.run(() -> io.drive(speeds.get()), this)
                .finallyDo(() -> io.drive(new ChassisVelocities()));
    }

    public void followChoreoTrajectory(SwerveSample sample) {
        // Add pose-error feedback to the path's field-relative velocity feedforward.
        // Wrapping heading error avoids commanding a full turn across the +/-pi boundary.
        ChassisVelocities requestedSpeeds = sample.getChassisSpeeds();

        requestedSpeeds.vx += DriveConstants.kTrajectoryXController.calculate(
                0.0, sample.x - getPose().getTranslation().getX());
        requestedSpeeds.vy += DriveConstants.kTrajectoryYController.calculate(
                0.0, sample.y - getPose().getTranslation().getY());
        requestedSpeeds.omega += DriveConstants.kTrajectoryThetaController.calculate(
                0.0,
                MathUtil.angleModulus(
                        sample.getPose()
                                .getRotation()
                                .minus(getPose().getRotation())
                                .getRadians()));

        Logger.recordOutput("Drive/Trajectory/SetpointPose", sample.getPose());
        Logger.recordOutput(
                "Drive/Trajectory/SetpointSpeeds", sample.getChassisSpeeds());

        io.drive(requestedSpeeds);
    }

    // public void addVisionMeasurement(VisionEstimate estimate) {
    //     io.addVisionMeasurement(estimate);
    // }
}
