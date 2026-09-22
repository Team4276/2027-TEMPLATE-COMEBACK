package frc.robot.controlboard;

import org.wpilib.units.Units;
import org.wpilib.units.measure.Time;

public class ControlBoardConstants {
	public static enum InputMode {
		CONTROLLER,
		KEYBOARD,
		DEMO
	}

	public static final InputMode kInputMode = InputMode.KEYBOARD;

	public static final int kDriverControllerPort = 0;
	public static final int kOperatorControllerPort = 1;

	public static final Time kIntakeRumbleTime = Units.Seconds.of(0.2);

	public static final double kStickDeadband = 0.05;
}
