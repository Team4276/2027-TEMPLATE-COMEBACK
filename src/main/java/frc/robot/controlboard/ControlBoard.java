package frc.robot.controlboard;

import org.wpilib.units.Units;
import org.wpilib.units.measure.Time;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.command2.button.CommandNiDsXboxController;
import frc.lib.hid.ViXController;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

/** Maps driver inputs to commands; configureBindings is called once after Robot initializes logging. */
public class ControlBoard extends SubsystemBase {
	public static final ControlBoard mInstance = new ControlBoard();

	public static final ViXController mDriver = new ViXController(
			ControlBoardConstants.kDriverControllerPort);
	public static final ViXController mOperator = new ViXController(
			ControlBoardConstants.kOperatorControllerPort);
	public static final CommandGenericHID mKeyboard0 = new CommandGenericHID(0);
	public static final CommandGenericHID mKeyboard1 = new CommandGenericHID(1);

	public void configureBindings() {
		// The default command can run whenever Drive is free, including autonomous gaps.
		// Gate joystick requests on teleop so those gaps cannot move the robot from stick input.
		Drive.mInstance.setDefaultCommand(Drive.mInstance.drive(() ->
				DriverStationBackend.isTeleopEnabled()
						? DriveConstants.kTeleopRequestUpdater.get()
						: new org.wpilib.math.kinematics.ChassisVelocities()));

		// Menu/Start matches the driver's controller diagram.
		mDriver.start()
				.onTrue(Commands.runOnce(
						() -> Drive.mInstance.zeroGyro(),
						Drive.mInstance)
						.ignoringDisable(true));

		// Keep the existing autonomous pose-reset request on the unused View/Back button.
		mDriver.back()
				.onTrue(Commands.runOnce(() -> Robot.resetPoseForAuto = true).ignoringDisable(true));

		// Skipping bindings alone leaves mechanism hardware unmanaged. Initialize every motor
		// with an explicit neutral request, then inhibit outputs when only testing the drivetrain.
		Superstructure.mInstance.stopAll();
		if (ControlBoardConstants.kDriveBringupMode) {
			Superstructure.mInstance.disableAll();
		} else {
			driverControls();
		}
		// bringupControls();
		// jogControls();
		// tuningControls();

		if (RobotConstants.getMode() == RobotConstants.Mode.SIM) {
			DriverStationBackend.silenceJoystickConnectionAlert(true);
		}
	}

	public void driverControls() {
		// Diagram: A = tower, Y = hub, X = ferry. Presets latch on a press;
		// right bumper retains the existing shooter/feed stop until auto-shot is implemented.
		// All driver mechanism actions are restricted to enabled teleop.
		mDriver.a().and(DriverStationBackend::isTeleopEnabled)
				.onTrue(Superstructure.mInstance.shootTower());
		mDriver.y().and(DriverStationBackend::isTeleopEnabled)
				.onTrue(Superstructure.mInstance.shootHub());
		mDriver.x().and(DriverStationBackend::isTeleopEnabled)
				.onTrue(Superstructure.mInstance.ferry());

		// Reserved from the diagram: RT = auto-shot/crawl, POV up/down = first/second
		// active period, POV left/right = enable/disable manual mode. These behaviors
		// are not implemented yet; do not bind them to unrelated mechanism actions.

		mDriver.rightBumper().and(DriverStationBackend::isTeleopEnabled)
				.onTrue(Superstructure.mInstance.idleFlywheels()
						.alongWith(Superstructure.mInstance.idleFeeders()));

		// LT deploys and runs the rollers; LB retracts with the rollers stopped.
		// Keep requirements while held and stop on release: deploy has no endpoint detection.
		mDriver.leftTrigger().and(DriverStationBackend::isTeleopEnabled)
				.whileTrue(Superstructure.mInstance.deployIntake());
		mDriver.leftBumper().and(DriverStationBackend::isTeleopEnabled)
				.whileTrue(Superstructure.mInstance.retractIntake());

		// B implements exhaust only; the diagram's turtle/trench behavior is still reserved.
		mDriver.b().and(DriverStationBackend::isTeleopEnabled)
				.whileTrue(Superstructure.mInstance.exhaustIntake());

	}

	public void bringupControls() {
	}

	public void jogControls() {
	}

	public void tuningControls() {
	}

	public Command rumbleCommand(Time duration) {
		return rumbleCommand(mDriver, duration);
	}

	public Command rumbleCommand(CommandNiDsXboxController controller, Time duration) {
		// The finalizer also clears rumble if another command interrupts the timed sequence.
		return Commands.sequence(
				Commands.runOnce(() -> {
					setRumble(controller, true);
				}),
				Commands.waitSeconds(duration.in(Units.Seconds)),
				Commands.runOnce(() -> {
					setRumble(controller, false);
				}))
				.finallyDo(() -> {
					setRumble(controller, false);
					;
				})
				.withName("Rumble");
	}

	public void setRumble(boolean on) {
		setRumble(mDriver, on);
	}

	public void setRumble(CommandNiDsXboxController controller, boolean on) {
		controller.getHID().setRumble(RumbleType.RIGHT_RUMBLE, on ? 1.0 : 0.0);
		controller.getHID().setRumble(RumbleType.LEFT_RUMBLE, on ? 1.0 : 0.0);
	}
}
