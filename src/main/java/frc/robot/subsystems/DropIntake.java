package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
    private final MotorControllerGroup dropdownMotors =
        new MotorControllerGroup(leftDropMotor, rightDropMotor);
    private final CANSparkMax intakeMotor =
        new CANSparkMax(Constants.DropDownIntake.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final DutyCycleEncoder dropEncoder =
        new DutyCycleEncoder(Constants.DropDownIntake.DROP_ENCODER_ID);
    private final PIDController pidController = new PIDController(Constants.DropDownIntake.PID.KP,
        Constants.DropDownIntake.PID.KI, Constants.DropDownIntake.PID.KD);
    private final ArmFeedforward feedforward = new ArmFeedforward(Constants.DropDownIntake.PID.KS,
        Constants.DropDownIntake.PID.KG, Constants.DropDownIntake.PID.KV);

    private final double dropEncoderOffset = 0.000;
    private final double defaultGoal = 0.0893 * 360;
    private final double coneDeployGoal = 0.000 * 360;
    private final double cubeDeployGoal = 0.0601 * 360;

    /**
     * Drop Down Intake Subsystem
     */
    public DropIntake() {
        leftDropMotor.setIdleMode(IdleMode.kBrake);
        rightDropMotor.setIdleMode(IdleMode.kBrake);
        rightDropMotor.setInverted(true);
    }

    /**
     * Deploy the dropdown intake to the specified height for a cone. Stops when within threshold.
     */
    public void intakeConeDeploy() {
        dropdownMotors.setVoltage(Constants.DropDownIntake.DROP_VOLTS);
    }

    /**
     * Deploy the dropdown intake to the specified height for a cube. Stops when within threshold.
     */
    public void intakeCubeDeploy() {
        dropdownMotors.setVoltage(Constants.DropDownIntake.DROP_VOLTS);
    }

    /**
     * Retracts the dropdown intake to the default height. Stops when within threshold.
     */
    public void intakeRetract() {
        dropdownMotors.setVoltage(Constants.DropDownIntake.RETRACT_VOLTS);
    }

    public void stopDrop() {
        dropdownMotors.setVoltage(Constants.DropDownIntake.STOP_VOLTS);
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
        return (dropEncoder.getAbsolutePosition() - dropEncoderOffset) * 360;
    }

    /**
     * Set the dropdown motors to go to a certain angle.
     *
     * @param angle Requested angle.
     */
    public void ddToAngle(double angle) {
        dropdownMotors.set(pidController.calculate(getAngleMeasurement(), angle)
            + feedforward.calculate((getAngleMeasurement() * (Math.PI / 180.0)), 0.0));
    }

    /**
     * Check if aligned with a requested goal.
     *
     * @param goal The requesed goal.
     * @return True if properly aligned, false if not.
     */
    public boolean checkIfAligned(double goal) {
        return Math.abs(getAngleMeasurement() - goal) < 5;
    }
}

