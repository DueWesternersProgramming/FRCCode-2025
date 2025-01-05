package frc.robot.sensors;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;

/**
 * The {@code CanCoder} class contains fields and methods pertaining to the function of the absolute encoder.
 */
public class CanCoder
{
	private CANcoder canCoder;
	private boolean inverted;
	private double positionOffset;

	public CanCoder(int port) {
		this.canCoder = new CANcoder(port);
		this.inverted = false;
		this.positionOffset = 0.0;
	}

	/**
	 * Returns the current raw position of the absolute encoder.
	 *
	 * @return the current raw position of the absolute encoder in radians.
	 */
	public double getPosition() {
		return (inverted ? -1.0 : 1.0) * Units.rotationsToRadians(canCoder.getAbsolutePosition().getValueAsDouble());
	}

	/**
	 * Inverts the absolute encoder.
	 *
	 * @param inverted flag indicating if inverted.
	 */
	public void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

	/**
	 * Sets the position offset between the raw position and the virtual position.
	 *
	 * @param offset offset in radians
	 */
	public void setPositionOffset(double offset) {
		positionOffset = offset;
	}

	/**
	 * Returns the position offset between the raw position and the virtual position.
	 *
	 * @return the position offset in radians.
	 */
	public double getPositionOffset() {
		return positionOffset;
	}

	/**
	 * Returns the virtual position of the absolute encoder (raw position minus offset).
	 *
	 * @return the virtual position in radians.
	 */
	public double getVirtualPosition() {
		return getPosition() - positionOffset;
	}

	/**
	 * Resets the virtual position to the current raw position.
	 */
	public void resetVirtualPosition() {
		positionOffset = getPosition();
	}

}
