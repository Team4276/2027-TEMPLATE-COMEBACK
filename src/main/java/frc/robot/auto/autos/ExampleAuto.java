package frc.robot.auto.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import org.wpilib.math.geometry.Pose2d;
import frc.robot.auto.AutoModeBase;

public class ExampleAuto extends AutoModeBase {

	AutoTrajectory startToFirstPOI = trajectory("startToFirstPOI");
	AutoTrajectory firstPOIToSecondPOI = trajectory("firstPOIToSecondPOI");

	public ExampleAuto(AutoFactory factory) {
		super(factory, "Example Auto");

		prepRoutine(
				startToFirstPOI.cmd(),
				// AutoHelpers.exampleCommand(),
				firstPOIToSecondPOI.cmd()
				// AutoHelpers.exampleCommand()
			);
	}

	@Override
	public Pose2d getInitialPose() {
		return startToFirstPOI.getInitialPose().get();
	}

}
