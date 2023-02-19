package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Creates the subsystem for the arm.
 */
public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor1 =
        new CANSparkMax(Constants.Arm.ARM_ID, MotorType.kBrushless);
    private final CANSparkMax armMotor2 =
        new CANSparkMax(Constants.Arm.ARM_ID_2, MotorType.kBrushless);
    // private final MotorControllerGroup armMotors = new MotorControllerGroup(armMotor1,
    // armMotor2);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(Constants.Arm.PID.K_SVOLTS,
        Constants.Arm.PID.K_GVOLTS, Constants.Arm.PID.K_WVOLT_SECOND_PER_RAD,
        Constants.Arm.PID.K_AVOLT_SECOND_SQUARED_PER_RAD);
    private final AbsoluteEncoder encoder1 = armMotor1.getAbsoluteEncoder(Type.kDutyCycle);
    private final AbsoluteEncoder encoder2 = armMotor2.getAbsoluteEncoder(Type.kDutyCycle);
    // private final ProfiledPIDController pid_controller =
    // new ProfiledPIDController(Constants.Arm.PID.KP, Constants.Arm.PID.KI, Constants.Arm.PID.KD,
    // new TrapezoidProfile.Constraints(Constants.Arm.PID.K_MAX_VELOCITY_RAD_PER_SECOND,
    // Constants.Arm.PID.K_MAX_ACCELERATION_RAD_PER_SEC_SQUARED));
    private final PIDController pid_controller1 =
        new PIDController(Constants.Arm.PID.KP, Constants.Arm.PID.KI, Constants.Arm.PID.KD);
    private final PIDController pid_controller2 =
        new PIDController(Constants.Arm.PID.KP, Constants.Arm.PID.KI, Constants.Arm.PID.KD);

    private final double encoder1Offset = 324.8828030;
    private final double encoder2Offset = 271.8351674;
    private double goalAngle;
    private boolean enablePID = false;

    /**
     * Arm Subsystem
     */
    public Arm() {
        armMotor1.restoreFactoryDefaults();
        armMotor2.restoreFactoryDefaults();
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor2.setIdleMode(IdleMode.kBrake);
        armMotor1.setInverted(false);
        armMotor2.setInverted(true);

        encoder1.setPositionConversionFactor(360);
        encoder1.setVelocityConversionFactor(360);
        encoder1.setInverted(true);
        encoder1.setZeroOffset(encoder1Offset);

        encoder2.setPositionConversionFactor(360);
        encoder2.setVelocityConversionFactor(360);
        encoder2.setInverted(false);
        encoder2.setZeroOffset(encoder2Offset);

        armMotor1.burnFlash();
        armMotor2.burnFlash();
    }

    @Override
    public void periodic() {
        if (enablePID) {
            armToAngle(goalAngle);
        }
    }

    /**
     * Set target angle for Arm
     *
     * @param goal Target Angel in Degrees
     */
    public void setGoal(double goal) {
        this.goalAngle = goal;

        pid_controller1.setSetpoint(goal);
        pid_controller2.setSetpoint(goal);
    }

    /**
     * Get the target angle for the Arm
     *
     * @return The target angle in degrees
     */
    public double getGoal() {
        return this.goalAngle;
    }

    /**
     * Moves the arm to specified angle and stops when done.
     *
     * @param angle requested angle in degrees
     */
    public void armToAngle(double angle) {
        armMotor1.setVoltage(pid_controller1.calculate(getAngleMeasurement1())
            + m_feedforward.calculate(Math.toRadians(getAngleMeasurement1() - 90), 0));
        armMotor2.setVoltage(pid_controller2.calculate(getAngleMeasurement2())
            + m_feedforward.calculate(Math.toRadians(getAngleMeasurement2() - 90), 0));
    }

    /**
     * Enable PID control
     */
    public void enablePID() {
        this.enablePID = true;
    }

    /**
     * Disable PID control
     *
     */
    public void disablePID() {
        this.enablePID = false;
    }

    /**
     * Get the current angle that is stated by the encoder.
     *
     * @return Current angle reported by encoder (0 - 360)
     */
    public double getAngleMeasurement1() {
        return encoder1.getPosition();
    }


    /**
     * Get the current angle that is stated by the encoder.
     *
     * @return Current angle reported by encoder (0 - 360)
     */
    public double getAngleMeasurement2() {
        return encoder2.getPosition();
    }

    /**
     * Check if aligned with a requested goal.
     *
     * @param goal The requesed goal in degrees.
     * @return True if properly aligned, false if not.
     */
    public boolean checkIfAligned1(double goal) {
        return Math.abs(getAngleMeasurement1() - goal) < 2;
    }

    /**
     * Check if aligned with a requested goal.
     *
     * @param goal The requesed goal in degrees.
     * @return True if properly aligned, false if not.
     */
    public boolean checkIfAligned2(double goal) {
        return Math.abs(getAngleMeasurement2() - goal) < 2;
    }
}
