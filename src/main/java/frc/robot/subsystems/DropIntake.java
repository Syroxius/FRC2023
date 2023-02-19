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
 * Creates the subsystem for the dropdown intake.
 */
public class DropIntake extends SubsystemBase {
    private final CANSparkMax leftDropMotor =
        new CANSparkMax(Constants.DropDownIntake.LEFT_DROP_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax rightDropMotor =
        new CANSparkMax(Constants.DropDownIntake.RIGHT_DROP_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax intakeMotor =
        new CANSparkMax(Constants.DropDownIntake.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final AbsoluteEncoder dropEncoder = leftDropMotor.getAbsoluteEncoder(Type.kDutyCycle);
    private final PIDController pidController = new PIDController(Constants.DropDownIntake.PID.KP,
        Constants.DropDownIntake.PID.KI, Constants.DropDownIntake.PID.KD);
    private final ArmFeedforward feedforwardLeft =
        new ArmFeedforward(Constants.DropDownIntake.PID.KS, Constants.DropDownIntake.PID.KG,
            Constants.DropDownIntake.PID.KV);
    private final ArmFeedforward feedforwardRight =
        new ArmFeedforward(Constants.DropDownIntake.PID.KS, Constants.DropDownIntake.PID.KG * .55,
            Constants.DropDownIntake.PID.KV);

    private final double dropEncoderOffset = 214.6783500;
    public final double position1 = 3;
    public final double position2 = 60;
    public final double position3 = 100;
    private double goalAngle = 0;
    private boolean enablePID = false;

    /**
     * Drop Down Intake Subsystem
     */
    public DropIntake() {
        leftDropMotor.restoreFactoryDefaults();
        rightDropMotor.restoreFactoryDefaults();
        leftDropMotor.setIdleMode(IdleMode.kBrake);
        rightDropMotor.setIdleMode(IdleMode.kBrake);
        leftDropMotor.setInverted(false);
        rightDropMotor.setInverted(true);
        dropEncoder.setPositionConversionFactor(360);
        dropEncoder.setVelocityConversionFactor(360);
        dropEncoder.setZeroOffset(dropEncoderOffset);
        leftDropMotor.burnFlash();
        rightDropMotor.burnFlash();
    }

    @Override
    public void periodic() {
        if (enablePID) {
            ddToAngle(goalAngle);
        }
    }

    /**
     * Set target angle for Drop Down Intake
     *
     * @param goal Target Angel in Degrees
     */
    public void setGoal(double goal) {
        this.goalAngle = goal;
    }

    /**
     * Get the target angle for the Drop Down Intake
     *
     * @return The target angle in degrees
     */
    public double getGoal() {
        return this.goalAngle;
    }

    /**
     * Set the dropdown motors to go to a certain angle.
     *
     * @param angle Requested angle.
     */
    public void ddToAngle(double angle) {
        leftDropMotor.setVoltage(pidController.calculate(getAngleMeasurement(), angle)
            + feedforwardLeft.calculate((Math.toRadians(getAngleMeasurement())), 0.0));
        rightDropMotor.setVoltage(pidController.calculate(getAngleMeasurement(), angle)
            + feedforwardRight.calculate((Math.toRadians(getAngleMeasurement())), 0.0));
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

    // Runs the intake at a specified speed.
    public void intake() {
        intakeMotor.set(Constants.DropDownIntake.INTAKE_SPEED);
    }

    // Stops the intake from running.
    public void stop() {
        intakeMotor.setVoltage(0);
    }

    /**
     * Get the current angle that is stated by the encoder.
     *
     * @return Current angle reported by encoder (0 - 360)
     */
    public double getAngleMeasurement() {
        return dropEncoder.getPosition();
    }

    /**
     * Check if aligned with a requested goal.
     *
     * @param goal The requesed goal in degrees.
     * @return True if properly aligned, false if not.
     */
    public boolean checkIfAligned(double goal) {
        return Math.abs(getAngleMeasurement() - goal) < 2;
    }
}

