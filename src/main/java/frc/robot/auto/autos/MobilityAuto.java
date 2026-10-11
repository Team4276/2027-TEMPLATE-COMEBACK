package frc.robot.auto.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import org.wpilib.math.geometry.Pose2d;

public class MobilityAuto extends AutoModeBase {
    private final AutoTrajectory mobility;

    public MobilityAuto(AutoFactory factory, String autoName, String trajectoryName) {
        super(factory, autoName);
        mobility = trajectory(trajectoryName);

        prepRoutine(
                mobility.resetOdometry(),
                mobility.cmd());
    }

    @Override
    public Pose2d getInitialPose() {
        return mobility.getInitialPose().orElseGet(Drive.mInstance::getPose);
    }
}