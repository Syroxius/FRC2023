package frc.lib.math;

import edu.wpi.first.math.controller.ElevatorFeedforward;

/**
 * A helper class that computes feedforward outputs for an elevator on a pivot (modeled as a motor
 * acting against the force of gravity).
 */
public class AngledElevatorFeedForward extends ElevatorFeedforward {

    /**
     * Creates a new ElevatorFeedforward with the specified gains. Units of the gain values will
     * dictate units of the computed feedforward.
     *
     * @param ks The static gain.
     * @param kg The gravity gain.
     * @param kv The velocity gain.
     */
    public AngledElevatorFeedForward(double ks, double kg, double kv) {
        super(ks, kg, kv);
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param positionRadians Angle of the elevator
     * @param velocity The velocity setpoint.
     * @param acceleration The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double positionRadians, double velocity, double acceleration) {
        return ks * Math.signum(velocity) + kg * Math.sin(positionRadians) + kv * velocity
            + ka * acceleration;
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param positionRadians Angle of the elevator
     * @param velocity The velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double positionRadians, double velocity) {
        return calculate(positionRadians, velocity, 0);
    }
}
