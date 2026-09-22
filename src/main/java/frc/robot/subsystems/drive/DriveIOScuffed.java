package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Celsius;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
// import frc.lib.util.vision.VisionEstimate;
import org.wpilib.units.measure.Temperature;

import frc.robot.subsystems.drive.scuffed.SwerveDrive;
import frc.robot.subsystems.drive.scuffed.SwerveModule;
import frc.robot.subsystems.drive.scuffed.SwerveModule.ModulePosition;

public class DriveIOScuffed implements DriveIO {
    private SwerveDrive mSwerveDrive;

    public DriveIOScuffed() {
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        mSwerveDrive.updateTelemetry();

        inputs.pose = mSwerveDrive.getPose();
        inputs.gyroAngle = mSwerveDrive.getGyroAngle();
        inputs.fieldRelativeSpeed = mSwerveDrive.getFieldRelativeVelocity();
        inputs.robotRelativeSpeed = mSwerveDrive.getRobotRelativeVelocity();

        inputs.modulesPositions = mSwerveDrive.getModulePositions();
        inputs.moduleStates = mSwerveDrive.getModuleVelocities();

        // Ha who needs exception handling
        inputs.module0Inputs = getFromModule(mSwerveDrive.getModule(ModulePosition.FRONT_LEFT));
        inputs.module1Inputs = getFromModule(mSwerveDrive.getModule(ModulePosition.FRONT_RIGHT));
        inputs.module2Inputs = getFromModule(mSwerveDrive.getModule(ModulePosition.BACK_LEFT));
        inputs.module3Inputs = getFromModule(mSwerveDrive.getModule(ModulePosition.BACK_RIGHT));
    }

    private ModuleInput getFromModule(SwerveModule module) {
        // return new ModuleInput(
        //         true,
        //         module.mDriveFx.getPosition(),
        //         module.mDriveFx.getVelocity(),
        //         module.mDriveFx.getMotorVoltage(),
        //         module.mDriveFx.getSupplyCurrent(),
        //         module.mDriveFx.getStatorCurrent(),
        //         Temperature.ofBaseUnits(0, Celsius),
        //         true,
        //         module.mTurnSpark.getPosition(),
        //         module.mTurnSpark.getVelocity(),
        //         module.mTurnSpark.getMotorVoltage(),
        //         module.mTurnSpark.getSupplyCurrent(),
        //         module.mTurnSpark.getStatorCurrent(),
        //         Temperature.ofBaseUnits(0, Celsius));
        return new ModuleInput(
                true,
                module.mDriveFx.getPosition().baseUnitMagnitude(),
                module.mDriveFx.getVelocity().baseUnitMagnitude(),
                module.mDriveFx.getMotorVoltage().baseUnitMagnitude(),
                module.mDriveFx.getSupplyCurrent().baseUnitMagnitude(),
                module.mDriveFx.getStatorCurrent().baseUnitMagnitude(),
                Temperature.ofBaseUnits(0, Celsius).baseUnitMagnitude(),
                true,
                module.mTurnSpark.getPosition().baseUnitMagnitude(),
                module.mTurnSpark.getVelocity().baseUnitMagnitude(),
                module.mTurnSpark.getMotorVoltage().baseUnitMagnitude(),
                module.mTurnSpark.getSupplyCurrent().baseUnitMagnitude(),
                module.mTurnSpark.getStatorCurrent().baseUnitMagnitude(),
                Temperature.ofBaseUnits(0, Celsius).baseUnitMagnitude());
    }

    @Override
    public void updateSim() {
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

    // @Override
    // public void addVisionMeasurement(VisionEstimate estimate) {
    //     mSwerveDrive.addVisionMeasurement(estimate.getPose(), estimate.getTimestamp().in(Seconds));
    // }
}