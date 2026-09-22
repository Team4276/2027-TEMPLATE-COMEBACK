package frc.lib.sim;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;
import org.wpilib.simulation.SingleJointedArmSim;

/**
 * Class for simulating a pivoting system powerd by one or more motors like a
 * rotating arm.
 */
public class PivotSim extends MechanismSim {
	protected final SingleJointedArmSim sim;

	/**
	 * Creates a PivotSim from provided constants.
	 *
	 * @param constants Constants to use for PivotSim.
	 */
	public PivotSim(PivotSimConstants constants) {
		super(constants.gearing);
		sim = new SingleJointedArmSim(
				constants.motor,
				constants.gearing,
				constants.momentOfInertia.in(Units.KilogramSquareMeters),
				constants.armLength.in(Units.Meters),
				constants.mechanismMinHardStop.in(Units.Radians),
				constants.mechanismMaxHardStop.in(Units.Radians),
				constants.simGravity,
				constants.mechanismStartPos.in(Units.Radians));
	}

	@Override
	public void setVoltage(Voltage voltage) {
		sim.setInputVoltage(voltage.in(Units.Volts));
	}

	/**
	 * Constants for creating a PivotSim.
	 */
	public static class PivotSimConstants {
		public DCMotor motor;
		public double gearing;
		public MomentOfInertia momentOfInertia;
		public Distance armLength;
		public Angle mechanismMinHardStop;
		public Angle mechanismMaxHardStop;
		public boolean simGravity;
		public Angle mechanismStartPos;
	}

	@Override
	public AngularVelocity getVelocity() {
		return Units.Radians.of(sim.getVelocity()).per(Units.Second);
	}

	@Override
	public Angle getPosition() {
		return Units.Radians.of(sim.getAngle());
	}

	@Override
	public Current getStatorCurrent() {
		return Units.Amps.of(sim.getCurrentDraw());
	}

	@Override
	protected void update(Time deltaTime) {
		sim.update(deltaTime.in(Units.Seconds));
	}

	@Override
	public void setState(Angle angle, AngularVelocity velocity) {
		sim.setState(angle.in(Units.Radians), velocity.in(Units.RadiansPerSecond));
	}
}
