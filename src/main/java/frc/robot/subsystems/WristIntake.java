package frc.robot.subsystems;

import java.util.Map;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

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

    private GenericEntry coneGrabbed = RobotContainer.mainDriverTab
        .add("Cone Grabbed", getConeSensor()).withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#FFFF00", "Color when false", "#FFFFFF"))
        .withPosition(10, 2).withSize(2, 1).getEntry();
    private GenericEntry cubeGrabbed = RobotContainer.mainDriverTab
        .add("Cube Grabbed", getConeSensor()).withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#A020F0", "Color when false", "#FFFFFF"))
        .withPosition(10, 3).withSize(2, 1).getEntry();
    private GenericEntry solenoidStatus = RobotContainer.mainDriverTab.add("Grabber Open", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "green", "Color when false", "red"))
        .withPosition(0, 4).withSize(2, 1).getEntry();

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
        closeGrabber();
    }

    @Override
    public void periodic() {
        coneGrabbed.setBoolean(getConeSensor());
        cubeGrabbed.setBoolean(getCubeSensor());
        solenoidStatus.setBoolean(this.wristSolenoid.get() == Value.kForward);
    }

    // Runs wrist intake motor to intake a game piece.
    public void intake() {
        wristIntakeMotors.set(Constants.Wrist.INTAKE_SPEED);
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
