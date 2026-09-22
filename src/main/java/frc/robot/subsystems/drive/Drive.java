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

public class Drive extends SubsystemBase {
    public static final Drive mInstance = new Drive();

    private DriveIO io = new DriveIOScuffed();
    private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private Drive() {
    }

    @Override
    public void periodic() {
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
        resetPose(
                new Pose2d(
                        getPose().getTranslation(),
                        AllianceFlipUtil.apply(Rotation2d.ZERO)));

    }

    public Command drive(ChassisVelocities speeds) {
        return drive(() -> speeds);
    }

    public Command drive(Supplier<ChassisVelocities> speeds) {
        return Commands.run(() -> io.drive(speeds.get()), this);
    }

    public void followChoreoTrajectory(SwerveSample sample) {
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