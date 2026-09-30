package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import org.wpilib.math.geometry.Pose2d;

import java.util.function.Supplier;
import java.nio.file.Files;
import org.wpilib.system.Filesystem;

import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import frc.robot.auto.autos.ExampleAuto;

public class AutoSelector {
    private AutoChooser mAutoChooser = new AutoChooser();
    private LoggedNetworkChooser<Supplier<AutoRoutine>> mNetworkChooser = new LoggedNetworkChooser<Supplier<AutoRoutine>>("AutoChooserAkit");

    private Pose2d startPose = new Pose2d();

    public AutoSelector(AutoFactory autoFactory) {
        Supplier<AutoRoutine> doNothing = () -> {
            startPose = new Pose2d();
            return autoFactory.newRoutine("Do Nothing");
        };
        mAutoChooser.addRoutine("Do Nothing", doNothing);
        mNetworkChooser.addDefault("Do Nothing", doNothing);

        // The template ships without paths. Only offer the example when both are deployed.
        var choreoDirectory = Filesystem.getDeployDirectory().toPath().resolve("choreo");
        if (Files.isRegularFile(choreoDirectory.resolve("startToFirstPOI.traj"))
                && Files.isRegularFile(choreoDirectory.resolve("firstPOIToSecondPOI.traj"))) {
            Supplier<AutoRoutine> example = () -> generateAuto(new ExampleAuto(autoFactory));
            mAutoChooser.addRoutine("Example Auto", example);
            mNetworkChooser.add("Example Auto", example);
        }
    }

    private AutoRoutine generateAuto(AutoModeBase auto) {
        startPose = auto.getInitialPose();
        return auto.getRoutine();
    }

    public Command getSelectedCommand() {
        // return mAutoChooser.selectedCommand();
        return mNetworkChooser.get().get().cmd();
    }

    public AutoChooser getAutoChooser() {
        return mAutoChooser;
    }

    public LoggedNetworkChooser<Supplier<AutoRoutine>> getNetworkChooser() {
        return mNetworkChooser;
    }

    public Pose2d getSelectedAutoStartingPose() {
        mNetworkChooser.get().get();
        return startPose;
    }
}
