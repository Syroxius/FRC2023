package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

    private final DigitalInput coneSensor1 = new DigitalInput(Constants.Wrist.CONE_SENSOR_ID_UPPER);
    private final DigitalInput coneSensor2 = new DigitalInput(Constants.Wrist.CONE_SENSOR_ID_LOWER);
    private final DigitalInput cubeSensor1 = new DigitalInput(Constants.Wrist.CUBE_SENSOR_ID_UPPER);
    private final DigitalInput cubeSensor2 = new DigitalInput(Constants.Wrist.CUBE_SENSOR_ID_LOWER);

    public WristIntake() {
        leftWristMotor.setInverted(true);
        wristIntakeMotors.setInverted(true);
    }

    // Runs wrist intake motor to intake a game piece.
    public void intake() {
        wristIntakeMotors.set(Constants.Wrist.INTAKE_SPEED);
    }

    // Runs wrist intake motor to gently spit out game piece.
    public void release() {
        wristIntakeMotors.set(Constants.Wrist.INTAKE_RELEASE_SPEED);
    }

    // Stops the wrist intake motor from running.
    public void stop() {
        wristIntakeMotors.set(Constants.Wrist.INTAKE_STOP_SPEED);
    }

    // Releases GPs in intake (if any).
    // Primarily used to prevent the accidental holding of multiple GPs penalty.
    public void panic() {
        wristIntakeMotors.set(Constants.Wrist.INTAKE_PANIC_SPEED);
    }

    // Get and return the status of the cone sensors.
    public boolean getConeSensor() {
        return coneSensor1.get() && coneSensor2.get();
    }

    // Get and return the status of the cube sensors.
    public boolean getCubeSensor() {
        return cubeSensor1.get() && cubeSensor2.get();
    }
}
