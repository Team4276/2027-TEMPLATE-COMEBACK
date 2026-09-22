package frc.lib.io;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import org.wpilib.units.Units;
import org.wpilib.system.Notifier;
import frc.lib.sim.MechanismSim;

/**
 * Class used to control a main TalonFX and any number of followers for a
 * simulated mechanism.
 */
public class MotorIOTalonFXSim extends MotorIOTalonFX {
	private MechanismSim sim;
	private Notifier notifier;

	/**
	 * Creates a MotorIOTalonFX from a provided configuration.
	 *
	 * @param config    Configuration to create MotorIOTalonFX from.
	 * @param simObject Object used to simulate mechanism.
	 */
	public MotorIOTalonFXSim(MotorIOTalonFXConfig config, MechanismSim simObject) {
		super(config);
		sim = simObject;

		notifier = new Notifier(() -> {
			runSimulation();
		});
		notifier.startPeriodic(0.005);
	}

	/**
	 * Gets a multipler to account for the motor inversion in the simulation state.
	 *
	 * @return -1.0 if output is inverted or 1.0 if not
	 */
	private double getMotorInvertMultiplier() {
		return config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive ? -1.0 : 1.0;
	}

	private void runSimulation() {
		sim.setVoltage(Units.Volts.of(main.getSimState().getMotorVoltage()).times(getMotorInvertMultiplier()));
		sim.simulate();
		updateSimStates();
	}

	@Override
	public void updateInputs() {
		inputs.enabled = getEnabled();
		inputs.setPointType = Mode.IDLE;
		inputs.setPointValueAsDouble = 0.0;

		inputs.position[0] = sim.getPosition().baseUnitMagnitude();
		inputs.velocity[0] = sim.getVelocity().baseUnitMagnitude();
		inputs.statorCurrent[0] = sim.getStatorCurrent().baseUnitMagnitude();
		inputs.supplyCurrent[0] = Units.Amps.of(main.getSimState().getSupplyCurrent()).times(getMotorInvertMultiplier())
				.baseUnitMagnitude();
		inputs.motorVoltage[0] = Units.Volts.of(main.getSimState().getMotorVoltage()).times(getMotorInvertMultiplier())
				.baseUnitMagnitude();

		// inputs.position[0] = sim.getPosition();
		// inputs.velocity[0] = sim.getVelocity();
		// inputs.statorCurrent[0] = sim.getStatorCurrent();
		// inputs.supplyCurrent[0] =
		// Units.Amps.of(main.getSimState().getSupplyCurrent()).times(getMotorInvertMultiplier());
		// inputs.motorVoltage[0] =
		// Units.Volts.of(main.getSimState().getMotorVoltage()).times(getMotorInvertMultiplier());
	}

	/**
	 * Updates the main motor's simulation state.
	 */
	private void updateSimStates() {
		main.getSimState()
				.setRotorVelocity(sim.mechanismToRotor(sim.getVelocity().times(getMotorInvertMultiplier())));
		main.getSimState()
				.setRawRotorPosition(sim.mechanismToRotor(sim.getPosition().times(getMotorInvertMultiplier())));
		main.getSimState().setSupplyVoltage(12.0);

		for (TalonFX follower : followers) {
			follower.getSimState()
					.setRotorVelocity(sim.mechanismToRotor(sim.getVelocity().times(getMotorInvertMultiplier())));
			follower.getSimState()
					.setRawRotorPosition(sim.mechanismToRotor(sim.getPosition().times(getMotorInvertMultiplier())));
			follower.getSimState().setSupplyVoltage(12.0);
		}
	}
}
