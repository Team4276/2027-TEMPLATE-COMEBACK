package frc.lib.util;

import static org.wpilib.units.Units.Value;

import org.wpilib.hardware.hal.util.BoundaryException;
import org.wpilib.units.BaseUnits;
import org.wpilib.units.measure.Time;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import java.util.function.Consumer;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

public class ControllerUtil {

	public static Setpoint createFromMode(Mode mode, double baseUnits) {
		return switch (mode) {
			case IDLE -> Setpoint.withCoastSetpoint();
			case VOLTAGE -> Setpoint.withVoltageSetpoint(BaseUnits.VoltageUnit.of(baseUnits));
			case MOTIONMAGIC -> Setpoint.withMotionMagicSetpoint(BaseUnits.AngleUnit.of(baseUnits));
			case VELOCITY -> Setpoint.withVelocitySetpoint(
					BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(baseUnits));
			case DUTY_CYCLE -> Setpoint.withDutyCycleSetpoint(Value.of(baseUnits));
			case POSITIONPID -> Setpoint.withPositionSetpoint(BaseUnits.AngleUnit.of(baseUnits));
		};
	}

	/**
	 *
	 * @param <IO>              MotorIO subsysten is extending
	 * @param subsystem         subsystem to bind jog to
	 * @param intervalBaseUnits intervals between each jog input
	 * @param initialValue      subsystem's start value of the jogging action
	 * @param maxValueBaseUnits max value to apply jogging action to
	 * @param minValueBaseUnits min value to apploy jogging action to
	 * @param setpointMode      mode to apply jogging action to
	 * @param startSetpoint     subsystem's initial setpoint
	 * @param continousDelay    delay between each jog input when holding down the
	 *                          trigger
	 * @param triggers          binded triggers ([0]: jog up (REQUIRED FOR AN EFFECT
	 *                          TO HAPPEN), [1]: jog down, [2]: set to start)
	 */
	public static <IO extends MotorIO> void bindJog(
			MotorSubsystem<IO> subsystem,
			double intervalBaseUnits,
			double initialValue,
			double maxValueBaseUnits,
			double minValueBaseUnits,
			Mode setpointMode,
			Setpoint startSetpoint,
			Time continousDelay,
			Trigger... triggers) {

		if (minValueBaseUnits > maxValueBaseUnits) {
			throw new BoundaryException("Min output can't be greater than the max output");
		}

		subsystem.applySetpoint(startSetpoint); // apply the setpoint in case the subsytem currently has no setpoint

		Consumer<Double> setpointOffseter = (offset) -> {
			Setpoint followed = subsystem.getSetpoint();
			Setpoint next;
			if (followed == startSetpoint) {
				next = createFromMode(setpointMode, initialValue + offset);
			} else {
				if (followed.mode != setpointMode) {
					Logger.recordOutput("Jogging/Last Unable to Jog Seconds timestamp", Timer.getMonotonicTimestamp());
					return;
				}
				if (followed.baseUnits + offset == initialValue) {
					next = startSetpoint;
				} else {
					double nextBaseUnits = followed.baseUnits + offset;
					next = createFromMode(
							setpointMode, Math.max(minValueBaseUnits, Math.min(nextBaseUnits, maxValueBaseUnits)));
				}
			}
			subsystem.applySetpoint(next);
		};

		Function<Double, Command> commandGetter = (interval) -> Commands
				.runOnce(() -> setpointOffseter.accept(interval))
				.andThen(Commands.waitTime(continousDelay))
				.repeatedly();

		switch (triggers.length) {
			case 3:
				triggers[2].onTrue(subsystem.setpointCommand(startSetpoint));
			case 2:
				triggers[1].whileTrue(commandGetter.apply(-intervalBaseUnits));
			case 1:
				triggers[0].whileTrue(commandGetter.apply(intervalBaseUnits));
			default:
				break;
		}
	}

	/**
	 *
	 * @param <IO>                 MotorIO subsysten is extending
	 * @param subsystem            subsystem to bind jog to
	 * @param intervalBaseUnits    intervals between each jog input
	 * @param initialValue         subsystem's start value of the jogging action
	 * @param maxAbsoluteBaseUnits max absolute value to apply jogging action to
	 * @param setpointMode         mode to apply jogging action to
	 * @param startSetpoint        subsystem's initial setpoint
	 * @param continousDelay       delay between each jog input when holding down
	 *                             the trigger
	 * @param triggers             binded triggers ([0]: jog up (REQUIRED FOR AN
	 *                             EFFECT TO HAPPEN), [1]: jog down, [2]: set to
	 *                             start)
	 */
	public static <IO extends MotorIO> void bindJog(
			MotorSubsystem<IO> subsystem,
			double intervalBaseUnits,
			double initialValue,
			double maxAbsoluteBaseUnits,
			Mode setpointMode,
			Setpoint startSetpoint,
			Time continousDelay,
			Trigger... triggers) {
		double ensuredAbsolute = Math.abs(maxAbsoluteBaseUnits);
		bindJog(
				subsystem,
				intervalBaseUnits,
				initialValue,
				ensuredAbsolute,
				-ensuredAbsolute,
				setpointMode,
				startSetpoint,
				continousDelay,
				triggers);
	}
}
