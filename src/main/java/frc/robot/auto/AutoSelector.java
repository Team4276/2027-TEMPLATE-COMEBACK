package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import org.wpilib.math.geometry.Pose2d;

import java.util.function.Supplier;
import java.nio.file.Files;
import org.wpilib.system.Filesystem;

import org.wpilib.command2.Command;
import frc.robot.auto.autos.ExampleAuto;
import frc.robot.auto.autos.MobilityAuto;

public class AutoSelector {
    private final AutoChooser mAutoChooser = new AutoChooser("Do Nothing");

    private Pose2d startPose = new Pose2d();

    public AutoSelector(AutoFactory autoFactory) {
        Supplier<AutoRoutine> doNothing = () -> {
            startPose = new Pose2d();
            return autoFactory.newRoutine("Do Nothing");
        };
        mAutoChooser.addRoutine("Do Nothing", doNothing);

        // The template ships without paths. Only offer the example when both are deployed.
        var choreoDirectory = Filesystem.getDeployDirectory().toPath().resolve("choreo");
        if (Files.isRegularFile(choreoDirectory.resolve("startToFirstPOI.traj"))
                && Files.isRegularFile(choreoDirectory.resolve("firstPOIToSecondPOI.traj"))) {
            Supplier<AutoRoutine> example = () -> generateAuto(new ExampleAuto(autoFactory));
            mAutoChooser.addRoutine("Example Auto", example);
        }

        registerMobilityAuto(autoFactory, choreoDirectory, "Red Mobility", "Red_Mobility");
        registerMobilityAuto(autoFactory, choreoDirectory, "Blue Mobility", "Blue_Mobility");
    }

    private void registerMobilityAuto(AutoFactory autoFactory, java.nio.file.Path choreoDirectory,
            String autoName, String trajectoryName) {
        if (!Files.isRegularFile(choreoDirectory.resolve(trajectoryName + ".traj"))) {
            return;
        }

        Supplier<AutoRoutine> mobility = () -> generateAuto(
                new MobilityAuto(autoFactory, autoName, trajectoryName));
        mAutoChooser.addRoutine(autoName, mobility);
    }

    private AutoRoutine generateAuto(AutoModeBase auto) {
        startPose = auto.getInitialPose();
        return auto.getRoutine();
    }

    public Command getSelectedCommand() {
        return mAutoChooser.selectedCommand();
    }

    public AutoChooser getAutoChooser() {
        return mAutoChooser;
    }

    public Pose2d getSelectedAutoStartingPose() {
        return startPose;
    }
}
