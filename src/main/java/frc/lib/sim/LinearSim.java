package frc.lib.sim;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.BaseUnits;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;
import org.wpilib.simulation.ElevatorSim;
import frc.lib.util.Util;

/**
 * Class for simulating a linear system powerd by one or more motors like an
 * elevator.
 */
public class LinearSim extends MechanismSim {
	private final Util.DistanceAngleConverter converter;
	protected final ElevatorSim sim;

	/**
	 * Creates a LinearSim from provided constants.
	 *
	 * @param constants Constants to use for LinearSim.
	 */
	public LinearSim(LinearSimConstants constants) {
		super(constants.gearing);
		this.converter = constants.converter;
		sim = new org.wpilib.simulation.ElevatorSim(
				constants.motor,
				constants.gearing,
				constants.carriageMass.in(Units.Kilograms),
				converter.getDrumRadius().in(Units.Meters),
				constants.minHeight.in(Units.Meters),
				constants.maxHeight.in(Units.Meters),
				constants.simGravity,
				constants.startingHeight.in(Units.Meters));
	}

	@Override
	public void setVoltage(Voltage voltage) {
		sim.setInputVoltage(voltage.in(Units.Volts));
	}

	/**
	 * Constants for creating a LinearSim.
	 */
	public static class LinearSimConstants {
		public Util.DistanceAngleConverter converter;
		public DCMotor motor;
		public double gearing;
		public Mass carriageMass;
		public Distance minHeight;
		public Distance maxHeight;
		public boolean simGravity;
		public Distance startingHeight;
	}

	@Override
	public AngularVelocity getVelocity() {
		return converter
				.toAngle(Units.Meters.of(sim.getVelocity()))
				.per(Units.Second);
	}

	@Override
	public Angle getPosition() {
		return converter.toAngle(Units.Meters.of(sim.getPosition()));
	}

	@Override
	public Current getStatorCurrent() {
		return Units.Amps.of(sim.getCurrentDraw());
	}

	@Override
	protected void update(Time deltaTime) {
		sim.update(deltaTime.in(Units.Second));
	}

	@Override
	public void setState(Angle angle, AngularVelocity velocity) {
		sim.setState(
				converter.toDistance(angle).in(Units.Meters),
				converter
						.toDistance(BaseUnits.AngleUnit.of(velocity.in(BaseUnits.AngleUnit.per(Units.Second))))
						.in(Units.Meters));
	}
}
