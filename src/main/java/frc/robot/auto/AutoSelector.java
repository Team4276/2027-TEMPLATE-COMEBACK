package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import org.wpilib.math.geometry.Pose2d;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import frc.robot.auto.autos.ExampleAuto;

public class AutoSelector {
    private AutoChooser mAutoChooser = new AutoChooser();
    private LoggedNetworkChooser<Supplier<AutoRoutine>> mNetworkChooser = new LoggedNetworkChooser<Supplier<AutoRoutine>>("AutoChooserAkit");

    private Pose2d startPose = new Pose2d();

    public AutoSelector(AutoFactory autoFactory) {
        mAutoChooser.addRoutine("Example Auto", () -> generateAuto(new ExampleAuto(autoFactory)));
        mAutoChooser.addRoutine("Do Nothing", () -> autoFactory.newRoutine("Do Nothing"));

        mNetworkChooser.add("Example Auto", () -> generateAuto(new ExampleAuto(autoFactory)));
        mNetworkChooser.add("Do Nothing", () -> autoFactory.newRoutine("Do Nothing"));
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
        return startPose;
    }
}
