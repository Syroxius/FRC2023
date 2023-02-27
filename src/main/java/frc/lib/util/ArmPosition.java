package frc.lib.util;

/**
 * Class representing the position of the Arm components
 */
public class ArmPosition {
    double armAngle;
    double elevatorPosition;
    double wristAngle = -10;

    /**
     * Class representing the position of the Arm components
     */
    public ArmPosition(double armAngle, double elevatorPosition, double wristAngle) {
        this.armAngle = armAngle;
        this.elevatorPosition = elevatorPosition;
        this.wristAngle = wristAngle;
    }

    /**
     * Class representing the position of the Arm components
     */
    public ArmPosition(double armAngle, double elevatorPosition) {
        this(armAngle, elevatorPosition, -10);
    }

    /**
     * Class representing the position of the Arm components
     */
    public ArmPosition(double armAngle) {
        this(armAngle, 0, -10);
    }

    /**
     * Get the Arm Angle
     *
     * @return Arm Angle
     */
    public double getArmAngle() {
        return armAngle;
    }

    /**
     * Get Elevator Position
     *
     * @return Elevator Position
     */
    public double getElevatorPosition() {
        return elevatorPosition;
    }

    /**
     * Get Wrist Angle
     *
     * @return Wrist Angle
     */
    public double getWristAngle() {
        return wristAngle;
    }

}
