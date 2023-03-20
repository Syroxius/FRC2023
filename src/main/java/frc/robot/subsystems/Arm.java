package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.DoubleJointedArmFeedforward;
import frc.robot.Constants;

/**
 * Creates the subsystem for the arm.
 */
public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor1 =
        new CANSparkMax(Constants.Arm.ARM_ID, MotorType.kBrushless);
    private final CANSparkMax armMotor2 =
        new CANSparkMax(Constants.Arm.ARM_ID_2, MotorType.kBrushless);
    private final AbsoluteEncoder armEncoder = armMotor1.getAbsoluteEncoder(Type.kDutyCycle);
    private final DoubleSolenoid armSolenoid;

    ProfiledPIDController armPIDController =
        new ProfiledPIDController(Constants.Arm.PID.kP, Constants.Arm.PID.kI, Constants.Arm.PID.kD,
            new TrapezoidProfile.Constraints(Constants.Arm.PID.MAX_VELOCITY,
                Constants.Arm.PID.MAX_ACCELERATION));

    // WRIST
    public final CANSparkMax wristMotor =
        new CANSparkMax(Constants.Wrist.WRIST_MOTOR_ID, MotorType.kBrushless);
    private final AbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

    ProfiledPIDController wristPIDController = new ProfiledPIDController(Constants.Wrist.PID.kP,
        Constants.Wrist.PID.kI, Constants.Wrist.PID.kD, new TrapezoidProfile.Constraints(
            Constants.Wrist.PID.MAX_VELOCITY, Constants.Wrist.PID.MAX_ACCELERATION));

    private DoubleJointedArmFeedforward feedforward =
        new DoubleJointedArmFeedforward(Constants.Arm.config, Constants.Wrist.config);

    private boolean enablePID = false;

    /**
     * Arm Subsystem
     */
    public Arm(PneumaticHub ph) {
        // ARM
        armMotor1.restoreFactoryDefaults();
        armMotor2.restoreFactoryDefaults();
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor2.setIdleMode(IdleMode.kBrake);
        armMotor1.setInverted(false);
        armMotor2.setInverted(true);
        armEncoder.setPositionConversionFactor(360);
        armEncoder.setVelocityConversionFactor(360);
        armEncoder.setInverted(true);
        armEncoder.setZeroOffset(Constants.Arm.encoder1Offset);

        armMotor1.burnFlash();
        armMotor2.burnFlash();
        // WRIST
        wristMotor.restoreFactoryDefaults();
        wristMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.setInverted(false);
        wristEncoder.setPositionConversionFactor(360);
        wristEncoder.setVelocityConversionFactor(360);
        wristEncoder.setInverted(false);
        wristEncoder.setZeroOffset(Constants.Wrist.encoderOffset);
        wristMotor.burnFlash();

        this.armSolenoid = ph.makeDoubleSolenoid(Constants.Arm.SOLENOID_FORWARD_CHANNEL,
            Constants.Arm.SOLENOID_REVERSE_CHANNEL);

        this.wristPIDController.setIntegratorRange(Constants.Wrist.PID.MIN_INTEGRAL,
            Constants.Wrist.PID.MAX_INTEGRAL);

        // armPIDController.enableContinuousInput(0, 2 * Math.PI);
        // armPidController2.enableContinuousInput(0, 2 * Math.PI);
        // wristPIDController.enableContinuousInput(0, 2 * Math.PI);

        enablePID = false;

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("arm", getArmAngle());
        SmartDashboard.putNumber("wrist", getWristAngle());
        SmartDashboard.putNumber("goal.arm", armPIDController.getGoal().position * 180 / Math.PI);
        SmartDashboard.putNumber("goal.wrist",
            wristPIDController.getGoal().position * 180 / Math.PI);

        if (enablePID) {
            double armState = armPIDController.calculate(getArmAngleRad());
            double wristState = wristPIDController.calculate(getWristAngleRad());

            SmartDashboard.putNumber("set.armPos", armState);
            SmartDashboard.putNumber("set.wristPos", wristState);

            var voltages =
                feedforward.calculate(VecBuilder.fill(getArmAngleRad(), getWristAngleRad()));

            SmartDashboard.putNumber("armFF", voltages.get(0, 0));
            SmartDashboard.putNumber("wristFF", voltages.get(1, 0));

            SmartDashboard.putNumber("armPID", armState);
            SmartDashboard.putNumber("wristPID", wristState);

            armMotor1.setVoltage(voltages.get(0, 0) + armState);
            armMotor2.setVoltage(voltages.get(0, 0) + armState);
            wristMotor.setVoltage(voltages.get(1, 0) + wristState);
        }

    }

    /**
     * Get angle of arm in degrees with a crossover outside of the configuration space
     */
    public double getArmAngle() {
        double angle = armEncoder.getPosition();
        if (angle > Constants.Arm.PID.TURNOVER_THRESHOLD) {
            angle -= 360;
        }
        return angle;
    }

    /**
     * Get angle of arm in radians with a crossover outside of the configuration space
     */
    public double getArmAngleRad() {
        return getArmAngle() * Math.PI / 180.0;
    }

    /**
     * Get angle of wrist in degrees with a crossover outside of the configuration space
     */
    public double getWristAngle() {
        double angle = 360 - wristEncoder.getPosition();
        if (angle > Constants.Wrist.PID.TURNOVER_THRESHOLD) {
            angle -= 360;
        }
        return angle;
    }

    /**
     * Get angle of wrist in radians with a crossover outside of the configuration space
     */
    public double getWristAngleRad() {
        return getWristAngle() * Math.PI / 180.0;
    }

    /**
     * Set setpoint for arm angle in degrees
     */
    public void setArmGoal(double goal) {
        enablePID = true;
        if (goal > Constants.Arm.PID.TURNOVER_THRESHOLD) {
            goal -= 360;
        }
        armPIDController.setGoal(goal * Math.PI / 180.0);
    }

    /**
     * Set setpoint for wrist angle in degrees
     */
    public void setWristGoal(double goal) {
        /*
         * if (goal > Constants.Wrist.PID.TURNOVER_THRESHOLD) { goal -= 360; }
         */
        wristPIDController.setGoal(goal * Math.PI / 180.0);
    }

    /**
     * Get if arm angle is close to the setpoint
     */
    public boolean armInPosition() {
        return armPIDController.atGoal();
    }

    /**
     * Get if wrist angle is close to the setpoint
     */
    public boolean wristInPosition() {
        return wristPIDController.atGoal();
    }

    /**
     * Set arm to extended position
     */
    public void extendArm() {
        armSolenoid.set(Value.kReverse);
    }

    /**
     * Set arm to retracted position
     */
    public void retractArm() {
        armSolenoid.set(Value.kForward);
    }
}
