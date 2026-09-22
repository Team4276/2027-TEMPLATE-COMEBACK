package frc.robot.controlboard;

import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Volts;

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
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.ControllerUtil;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.examplesubsystem.ExampleSubsystem;
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

		mDriver.back()
		.onTrue(Commands.runOnce(
		() -> Drive.mInstance.zeroGyro(),
		Drive.mInstance)
		.ignoringDisable(true));

		mDriver.start()
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
		// mKeyboard0.button(1)
		// .onTrue(Superstructure.mInstance.exampleCommand().onlyWhile(mKeyboard0.button(1)));
		// mKeyboard0.button(2)
		// .onTrue(Superstructure.mInstance.testCommand().onlyWhile(mKeyboard0.button(2)));

		mDriver.a()
				.onTrue(Superstructure.mInstance.setOn());

		mDriver.b()
				.onTrue(Superstructure.mInstance.setOff());
	}

	public void bringupControls() {
		mDriver.a().onTrue(ExampleSubsystem.mInstance.setpointCommand(ExampleSubsystem.EXAMPLE_SETPOINT));
	}

	public void jogControls() {
		ControllerUtil.bindJog(
				ExampleSubsystem.mInstance,
				Volts.of(0.01).baseUnitMagnitude(),
				Volts.of(0.0).baseUnitMagnitude(),
				Volts.of(12.0).baseUnitMagnitude(),
				Mode.VOLTAGE,
				Setpoint.withNeutralSetpoint(),
				Milliseconds.of(50.0),
				mOperator.a(),
				mOperator.b(),
				mOperator.rightBumper());
	}

	public void tuningControls() {
	}

	public Command rumbleCommand(Time duration) {
		return rumbleCommand(mDriver, duration);
	}

	public Command rumbleCommand(CommandNiDsXboxController controller, Time duration) {
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
