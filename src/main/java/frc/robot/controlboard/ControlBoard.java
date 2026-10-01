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

		mDriver.back()
				.onTrue(Commands.runOnce(
						() -> Drive.mInstance.zeroGyro(),
						Drive.mInstance)
						.ignoringDisable(true));

		mDriver.start()
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
		// These mechanism commands send persistent setpoints once. Releasing a button does not
		// itself neutralize the motor; explicit idle commands below replace the previous request.
		// Shooter/Feeder Controls
		mDriver.a()
				.whileTrue(Superstructure.mInstance.shootHub());
		mDriver.y()
				.whileTrue(Superstructure.mInstance.shootHubFar());
		mDriver.x()
				.whileTrue(Superstructure.mInstance.ferry());

		mDriver.rightTrigger()
				.whileTrue(Superstructure.mInstance.feed());

		mDriver.rightBumper()
				.whileTrue(Superstructure.mInstance.idleFlywheels()
						.alongWith(Superstructure.mInstance.idleFeeders()));

		// Intake Controls
		mDriver.leftTrigger()
				.whileTrue(Superstructure.mInstance.runIntake());
		mDriver.leftBumper()
				.whileTrue(Superstructure.mInstance.exhaustIntake());

		mDriver.b()
				.whileTrue(Superstructure.mInstance.retractIntake());
		mDriver.getHID().povUp()
				.whileTrue(Superstructure.mInstance.deployIntake());

		mDriver.leftTrigger().negate()
				// Idle the intake only when none of its mutually competing actions is requested.
				.and(mDriver.leftBumper().negate())
				.and(mDriver.b().negate())
				.and(mDriver.getHID().povUp().negate())
				.whileTrue(Superstructure.mInstance.idleIntake());

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
