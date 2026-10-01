package frc.robot.subsystems.drive;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.Units;
// import frc.lib.util.vision.VisionEstimate;

import frc.robot.subsystems.drive.scuffed.SwerveDrive;
import frc.robot.subsystems.drive.scuffed.SwerveModule;
import frc.robot.subsystems.drive.scuffed.SwerveModule.ModulePosition;

/** Adapts the local four-module swerve implementation to the logged DriveIO schema. */
public class DriveIOScuffed implements DriveIO {
    private final SwerveDrive mSwerveDrive = new SwerveDrive();

    public DriveIOScuffed() {
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        // Refresh sensors and odometry once before copying a consistent loop's telemetry.
        mSwerveDrive.updateTelemetry();

        inputs.pose = mSwerveDrive.getPose();
        inputs.gyroAngle = mSwerveDrive.getGyroAngle();
        inputs.fieldRelativeSpeed = mSwerveDrive.getFieldRelativeVelocity();
        inputs.robotRelativeSpeed = mSwerveDrive.getRobotRelativeVelocity();

        inputs.modulesPositions = mSwerveDrive.getModulePositions();
        inputs.moduleStates = mSwerveDrive.getModuleVelocities();
        inputs.moduleTargets = new SwerveModuleVelocity[4];

        inputs.module0Inputs = getFromModule(mSwerveDrive.getModule(ModulePosition.FRONT_LEFT));
        inputs.module1Inputs = getFromModule(mSwerveDrive.getModule(ModulePosition.FRONT_RIGHT));
        inputs.module2Inputs = getFromModule(mSwerveDrive.getModule(ModulePosition.BACK_LEFT));
        inputs.module3Inputs = getFromModule(mSwerveDrive.getModule(ModulePosition.BACK_RIGHT));
        for (ModulePosition position : ModulePosition.values()) {
            var module = mSwerveDrive.getModule(position);
            inputs.driveConfigFailed[position.ordinal()] = module.mDriveFx.inputs.configFailed;
            inputs.turnConfigFailed[position.ordinal()] = module.mTurnSpark.inputs.configFailed;
            int index = position.ordinal();
            inputs.moduleTargets[index] = module.getTargetVelocity();
            inputs.absoluteEncoderDegrees[index] = module.mTurnSpark.getPosition().in(Units.Degrees);
            inputs.steeringErrorDegrees[index] = Math.toDegrees(MathUtil.angleModulus(
                    module.getTargetVelocity().angle.minus(inputs.moduleStates[index].angle).getRadians()));
            inputs.appliedTargetMetersPerSecond[index] = module.getAppliedTargetMetersPerSecond();
        }
    }

    private ModuleInput getFromModule(SwerveModule module) {
        return new ModuleInput(
                module.mDriveFx.inputs.connected,
                module.mDriveFx.getPosition().baseUnitMagnitude(),
                module.mDriveFx.getVelocity().baseUnitMagnitude(),
                module.mDriveFx.getMotorVoltage().baseUnitMagnitude(),
                module.mDriveFx.getSupplyCurrent().baseUnitMagnitude(),
                module.mDriveFx.getStatorCurrent().baseUnitMagnitude(),
                module.mDriveFx.inputs.motorTemperature[0],
                module.mTurnSpark.inputs.connected,
                module.mTurnSpark.getPosition().baseUnitMagnitude(),
                module.mTurnSpark.getVelocity().baseUnitMagnitude(),
                module.mTurnSpark.getMotorVoltage().baseUnitMagnitude(),
                module.mTurnSpark.getSupplyCurrent().baseUnitMagnitude(),
                module.mTurnSpark.getStatorCurrent().baseUnitMagnitude(),
                module.mTurnSpark.inputs.motorTemperature[0]);
    }

    @Override
    public void updateSim() {
        // Placeholder: this adapter currently has no drivetrain physics integration.
        // mSwerveDrive.simIterate();
    }

    @Override
    public void resetPose(Pose2d pose) {
        mSwerveDrive.resetOdometry(pose);
    }

    @Override
    public void drive(ChassisVelocities velocities) {
        mSwerveDrive.setFieldRelativeChassisVelocities(velocities);
    }

    @Override
    public void driveRobotRelative(ChassisVelocities velocities) {
        mSwerveDrive.setRobotRelativeChassisVelocities(velocities);
    }

    // @Override
    // public void addVisionMeasurement(VisionEstimate estimate) {
    //     mSwerveDrive.addVisionMeasurement(estimate.getPose(), estimate.getTimestamp().in(Seconds));
    // }
}
