package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.units.Units;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.game.FieldLayout;
import java.util.List;

public class AutoModeBase {
	private AutoRoutine routine;

	private final String name;

	public AutoModeBase(AutoFactory factory, String name) {
		this.name = name;
		routine = factory.newRoutine(this.name);
	}

	public AutoTrajectory mirroredTrajectory(AutoTrajectory trajectory) {
		return mirroredTrajectoryFromRaw(trajectory.<SwerveSample>getRawTrajectory());
	}

	public AutoTrajectory mirroredTrajectory(String name) {
		return mirroredTrajectoryFromRaw(trajectory(name).<SwerveSample>getRawTrajectory());
	}

	private AutoTrajectory mirroredTrajectoryFromRaw(Trajectory<SwerveSample> trajectory) {
		return routine.trajectory(mirrorChoreoTrajectoryAcrossMidline(trajectory));
	}

	public static Trajectory<SwerveSample> mirrorChoreoTrajectoryAcrossMidline(Trajectory<SwerveSample> trajectory) {
		return mirrorSwerveTrajectoryAcrossY(trajectory);
	}

	private static Trajectory<SwerveSample> mirrorSwerveTrajectoryAcrossY(Trajectory<SwerveSample> trajectory) {
		final double fieldWidthMeters = FieldLayout.kFieldWidth.in(Units.Meters);
		List<SwerveSample> mirroredSamples = trajectory.samples().stream()
				.map(sample -> mirrorSwerveSampleAcrossY(sample, fieldWidthMeters))
				.toList();
		return new Trajectory<>(trajectory.name(), mirroredSamples, trajectory.splits(), trajectory.events());
	}

	private static SwerveSample mirrorSwerveSampleAcrossY(SwerveSample sample, double fieldWidthMeters) {
		double[] fx = sample.moduleForcesX();
		double[] fy = sample.moduleForcesY();
		double[] mirroredFx = new double[] { fx[1], fx[0], fx[3], fx[2] };
		double[] mirroredFy = new double[] { -fy[1], -fy[0], -fy[3], -fy[2] };

		return new SwerveSample(
				sample.t,
				sample.x,
				fieldWidthMeters - sample.y,
				-sample.heading,
				sample.vx,
				-sample.vy,
				-sample.omega,
				sample.ax,
				-sample.ay,
				-sample.alpha,
				mirroredFx,
				mirroredFy);
	}

	/**
	 * @return Trajectory from choreo
	 */
	public AutoTrajectory trajectory(String name) {
		return routine.trajectory(name);
	}

	public AutoTrajectory trajectory(String name, int index) {
		return routine.trajectory(name, index);
	}

	public <SampleType extends TrajectorySample<SampleType>> AutoTrajectory trajectory(
			Trajectory<SampleType> trajectory) {
		return routine.trajectory(trajectory);
	}

	public Pose2d getInitialPose() {
		return new Pose2d();
	}

	public void prepRoutine(Command... steps) {
		routine.active().onTrue(Commands.sequence(steps).withName("Auto Routine Sequential Command Group"));
	}

	public AutoRoutine getRoutine() {
		return routine;
	}

	public Command asCommand() {
		return routine.cmd();
	}
}
