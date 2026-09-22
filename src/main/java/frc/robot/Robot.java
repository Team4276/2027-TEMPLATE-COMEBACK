// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import choreo.auto.AutoFactory;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import frc.lib.util.LoggedTracer;
import frc.robot.auto.AutoSelector;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.drive.Drive;

public class Robot extends LoggedRobot {
    private AutoFactory mAutoFactory;
    private Command mAutonomousCommand;
    private final AutoSelector mAutoSelector;

    public static boolean resetPoseForAuto = false;

    public Robot() {
        Logger.recordMetadata("ProjectName", "RobotName"); // Set a metadata value

        // Set up data receivers & replay source
        switch (RobotConstants.getMode()) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();

        mAutoFactory = new AutoFactory(
                Drive.mInstance::getPose,
                Drive.mInstance::resetPose,
                Drive.mInstance::followChoreoTrajectory,
                true,
                Drive.mInstance);

        mAutoSelector = new AutoSelector(mAutoFactory);

        // Log active commands
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput(
                    "Commands/Unique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("Commands/All/" + name, count > 0);
        };

        CommandScheduler.getInstance()
                .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
        CommandScheduler.getInstance().onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
        CommandScheduler.getInstance()
                .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

        CommandScheduler.getInstance().schedule(mAutoFactory.warmupCmd());

        ControlBoard.mInstance.configureBindings();

        // RobotController.setBrownoutVoltages(Volts.of(5.5));

        // SmartDashboard.putData("Auto Selector", mAutoSelector.getAutoChooser());
    }

    @Override
    public void robotPeriodic() {
        LoggedTracer.reset();

        try {
            // Threads.setCurrentThreadPriority(true, 10);

            CommandScheduler.getInstance().run();
            // Threads.setCurrentThreadPriority(false, 0);
        } catch (Exception e) {
            Logger.recordOutput("Error/Last Loop Error/Last Error Message", e.getMessage());
            Logger.recordOutput("Error/Last Loop Error/Last Error Timestamp", Timer.getMonotonicTimestamp()); // TODO:
                                                                                                              // check
                                                                                                              // if this
                                                                                                              // is
                                                                                                              // correct
                                                                                                              // time
        }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        if (resetPoseForAuto) {
            Drive.mInstance.resetPose(mAutoSelector.getSelectedAutoStartingPose());
            resetPoseForAuto = false;
        }
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        mAutonomousCommand = mAutoSelector.getSelectedCommand();

        if (mAutonomousCommand != null) {
            CommandScheduler.getInstance().schedule(mAutonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
