package frc.lib.util;

/**
 * Class representing the position of the Arm components
 */
public class ArmPosition {
    double armAngle;
    boolean armExtended;
    double wristAngle = -10;

    /**
     * Class representing the position of the Arm components
     */
    public ArmPosition(double armAngle, boolean armExtended, double wristAngle) {
        this.armAngle = armAngle;
        this.armExtended = armExtended;
        this.wristAngle = wristAngle;
    }

    /**
     * Class representing the position of the Arm components
     */
    public ArmPosition(double armAngle, boolean armExtended) {
        this(armAngle, armExtended, -10);
    }

    /**
     * Class representing the position of the Arm components
     */
    public ArmPosition(double armAngle) {
        this(armAngle, false, -10);
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
     * Get Arm Extension
     *
     * @return True if extended, false if not
     */
    public boolean getArmExtended() {
        return armExtended;
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
