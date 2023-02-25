package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Creates the subsystem for the wrist intake.
 */
public class WristIntake extends SubsystemBase {

    private final CANSparkMax leftWristMotor =
        new CANSparkMax(Constants.Wrist.LEFT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax rightWristMotor =
        new CANSparkMax(Constants.Wrist.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final MotorControllerGroup wristIntakeMotors =
        new MotorControllerGroup(leftWristMotor, rightWristMotor);
    private final DoubleSolenoid wristSolenoid;


    private final DigitalInput coneSensor1 = new DigitalInput(Constants.Wrist.CONE_SENSOR_ID);
    private final DigitalInput cubeSensor1 = new DigitalInput(Constants.Wrist.CUBE_SENSOR_ID_LEFT);
    private final DigitalInput cubeSensor2 = new DigitalInput(Constants.Wrist.CUBE_SENSOR_ID_RIGHT);

    /**
     * Create Wrist Intake Subsystem
     *
     * @param ph Pnuematic Hub instance
     */
    public WristIntake(PneumaticHub ph) {
        leftWristMotor.setInverted(false);
        rightWristMotor.setInverted(true);
        this.wristSolenoid = ph.makeDoubleSolenoid(Constants.Wrist.SOLENOID_FORWARD_CHANNEL,
            Constants.Wrist.SOLENOID_REVERSE_CHANNEL);
        this.wristSolenoid.set(Value.kForward);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("cone", getConeSensor());
        SmartDashboard.putBoolean("cube1", cubeSensor1.get());
        SmartDashboard.putBoolean("cube2", cubeSensor2.get());
    }

    /**
     * Set power of intake motors
     *
     * @param power power of motors from -1 to 1
     */
    public void setMotors(double power) {
        wristIntakeMotors.set(power);
    }

    /**
     * Releases GPs in intake (if any). Primarily used to prevent the accidental holding of multiple
     * GPs penalty.
     */
    public void panic() {
        setMotors(Constants.Wrist.INTAKE_PANIC_SPEED);
    }

    /**
     * Get and return the status of the cone sensors.
     *
     * @return status of cone touch sensor
     */
    public boolean getConeSensor() {
        return coneSensor1.get();
    }

    /**
     * Get and return the status of the cube sensors.
     *
     * @return status of cube touch sensors
     */
    public boolean getCubeSensor() {
        return cubeSensor1.get() && cubeSensor2.get();
    }

    /**
     * Open grabber
     */
    public void openGrabber() {
        this.wristSolenoid.set(Value.kReverse);
    }

    /**
     * Close grabber
     */
    public void closeGrabber() {
        this.wristSolenoid.set(Value.kForward);
    }
}
