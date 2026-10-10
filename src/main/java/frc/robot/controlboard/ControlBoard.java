package frc.robot.controlboard;

import org.wpilib.units.Units;
import org.wpilib.units.measure.Time;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.command2.button.CommandXboxController;
import frc.lib.hid.ViXController;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.IntakeDeploy;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.superstructure.Superstructure;

public class ControlBoard extends SubsystemBase {
	public static final ControlBoard mInstance = new ControlBoard();

	public static final ViXController mDriver = new ViXController(
			ControlBoardConstants.kDriverControllerPort);
	public static final ViXController mOperator = new ViXController(
			ControlBoardConstants.kOperatorControllerPort);
	public static final CommandGenericHID mKeyboard0 = new CommandGenericHID(0);
	public static final CommandGenericHID mKeyboard1 = new CommandGenericHID(1);

	public void configureBindings() {
		Drive.mInstance.setDefaultCommand(Drive.mInstance.drive(DriveConstants.kTeleopRequestUpdater));

		mDriver.menu()
				.onTrue(Commands.runOnce(
						() -> Drive.mInstance.zeroGyro(),
						Drive.mInstance)
						.ignoringDisable(true));

		mDriver.view()
				.onTrue(Commands.runOnce(() -> Robot.resetPoseForAuto = true).ignoringDisable(true));

		driverControls();
		// bringupControls();
		// jogControls();
		// tuningControls();

		if (RobotConstants.getMode() == RobotConstants.Mode.SIM) {
			DriverStationBackend.silenceJoystickConnectionAlert(true);
		}
	}

	public void driverControls() {
		// Shooter/Feeder Controls
		mDriver.a()
				.onTrue(Superstructure.mInstance.shootHub());
		mDriver.y()
				.onTrue(Superstructure.mInstance.shootHubFar());
		mDriver.x()
				.onTrue(Superstructure.mInstance.ferry());

		// Keep feeding only while held; releasing the trigger ends the feed command.
		mDriver.rightTrigger()
				.whileTrue(Superstructure.mInstance.feed());
		mDriver.dpadDown()
				.whileTrue(Superstructure.mInstance.exhaust());

		mDriver.rightBumper()
				.onTrue(Superstructure.mInstance.idleFlywheels()
						.alongWith(Superstructure.mInstance.idleFeeders()));

		// Intake Controls
		mDriver.leftTrigger()
				.onTrue(Superstructure.mInstance.setIntakeRollers(IntakeRollers.INTAKE));
		mDriver.leftBumper()
				.onTrue(Superstructure.mInstance.setIntakeRollers(IntakeRollers.EXHAUST));
		mDriver.leftTrigger().negate()
				.and(mDriver.leftBumper().negate())
				.onTrue(Superstructure.mInstance.setIntakeRollers(IntakeRollers.IDLE));

		mDriver.b()
				.whileTrue(Superstructure.mInstance.setIntakeDeploy(IntakeDeploy.STOW));
		mDriver.dpadUp()
				.whileTrue(Superstructure.mInstance.setIntakeDeploy(IntakeDeploy.DEPLOY));
		mDriver.b().negate()
				.and(mDriver.dpadUp())
				.onFalse(Superstructure.mInstance.setIntakeDeploy(IntakeDeploy.IDLE));
		mDriver.dpadUp().negate()
				.and(mDriver.b())
				.onFalse(Superstructure.mInstance.setIntakeDeploy(IntakeDeploy.IDLE));
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

	public Command rumbleCommand(CommandXboxController controller, Time duration) {
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

	public void setRumble(CommandXboxController controller, boolean on) {
		controller.getHID().setRumble(RumbleType.RIGHT_RUMBLE, on ? 1.0 : 0.0);
		controller.getHID().setRumble(RumbleType.LEFT_RUMBLE, on ? 1.0 : 0.0);
	}
}
