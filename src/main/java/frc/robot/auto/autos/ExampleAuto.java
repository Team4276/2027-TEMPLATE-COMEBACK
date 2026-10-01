package frc.robot.auto.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import org.wpilib.math.geometry.Pose2d;
import frc.robot.auto.AutoModeBase;

/**
 * Two-path example requiring matching .traj files in deploy/choreo. Reset odometry before the
 * first segment so trajectory feedback starts in the path's coordinate frame.
 */
public class ExampleAuto extends AutoModeBase {

	AutoTrajectory startToFirstPOI = trajectory("startToFirstPOI");
	AutoTrajectory firstPOIToSecondPOI = trajectory("firstPOIToSecondPOI");

	public ExampleAuto(AutoFactory factory) {
		super(factory, "Example Auto");

		prepRoutine(
				startToFirstPOI.resetOdometry(),
				startToFirstPOI.cmd(),
				// AutoHelpers.exampleCommand(),
				firstPOIToSecondPOI.cmd()
				// AutoHelpers.exampleCommand()
			);
	}

	@Override
	public Pose2d getInitialPose() {
		return startToFirstPOI.getInitialPose().orElseGet(frc.robot.subsystems.drive.Drive.mInstance::getPose);
	}

}
