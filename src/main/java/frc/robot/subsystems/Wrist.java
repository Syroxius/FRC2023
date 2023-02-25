package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Creates new Subsystem and methods for the Wrist
 */
public class Wrist extends SubsystemBase {
    public final CANSparkMax wristMotor =
        new CANSparkMax(Constants.Wrist.WRIST_MOTOR_ID, MotorType.kBrushless);
    private final ArmFeedforward m_feedforward =
        new ArmFeedforward(Constants.Wrist.PID.kS, Constants.Wrist.PID.kG, Constants.Wrist.PID.kV);
    private final AbsoluteEncoder encoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

    private final PIDController pidController =
        new PIDController(Constants.Wrist.PID.kP, Constants.Wrist.PID.kI, Constants.Wrist.PID.kD);
    private final double encoderOffset = 61.0937476;
    public double lastAngle = 0;

    /**
     * Creates a new ProfilePIDController
     */
    public Wrist() {
        wristMotor.restoreFactoryDefaults();
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setInverted(false);
        encoder.setPositionConversionFactor(360);
        encoder.setVelocityConversionFactor(360);
        encoder.setInverted(false);
        encoder.setZeroOffset(encoderOffset);
        wristMotor.burnFlash();
        pidController.setTolerance(2);
        pidController.setSetpoint(90);
    }


    @Override
    public void periodic() {}

    /**
     * Test function
     */
    public void test() {
        double angle = getAngleMeasurement();
        double pidVol = pidController.calculate(lastAngle);
        double ff = m_feedforward.calculate(Math.toRadians(lastAngle - 90), 0);
        if (Math.abs(angle - lastAngle) < 2 || lastAngle == 0) {
            wristMotor.setVoltage(pidVol + ff);

        }
        SmartDashboard.putNumber("PID Voltage", pidVol);
        SmartDashboard.putNumber("FF Voltage", ff);
        SmartDashboard.putNumber("Wrist Encoder", getAngleMeasurement());
        lastAngle = angle;
    }

    /**
     * @return the rotation of the Wrist Encoder
     */
    public double getAngleMeasurement() {
        return encoder.getPosition();
    }
}
