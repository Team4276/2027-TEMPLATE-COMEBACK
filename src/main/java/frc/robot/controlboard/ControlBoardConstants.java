package frc.robot.controlboard;

import org.wpilib.units.Units;
import org.wpilib.units.measure.Time;

/** Operator interface settings; USB ports are Driver Station slots, independent of motor CAN IDs. */
public class ControlBoardConstants {
    // Optional drivetrain-only mode: inhibits mechanisms and skips their bindings.
    public static final boolean kDriveBringupMode = false;
    // Keep the reduced driving speed independently of whether mechanism controls are enabled.
    public static final double kDriveSpeedScale = 0.2;
	public static enum InputMode {
		CONTROLLER,
		KEYBOARD,
		DEMO
	}

	// DEMO currently selects the same drive supplier as CONTROLLER; kDriveSpeedScale sets the cap.
	public static final InputMode kInputMode = InputMode.CONTROLLER;

	public static final int kDriverControllerPort = 0;
	public static final int kOperatorControllerPort = 1;

	public static final Time kIntakeRumbleTime = Units.Seconds.of(0.2);

	// Reserved value: current ControlBoard construction uses ViXController's 0.1 default instead.
	public static final double kStickDeadband = 0.05;
}
