package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Creates the subsystem for the wrist intake.
 */
public class WristIntake extends SubsystemBase {

    private final WPI_TalonFX wristIntakeMotor = new WPI_TalonFX(Constants.Wrist.WRIST_INTAKE_ID);

    private boolean invertForCone = true;

    /**
     * Create Wrist Intake Subsystem
     */
    public WristIntake() {
        wristIntakeMotor.setInverted(false);
        wristIntakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Intake Current", wristIntakeMotor.getStatorCurrent());
    }

    // Runs wrist intake motor to intake a game piece.
    public void intake() {
        wristIntakeMotor.set(ControlMode.PercentOutput, Constants.Wrist.INTAKE_SPEED);
    }

    /**
     * Set power of intake motors
     *
     * @param power power of motors from -1 to 1
     */
    public void setMotor(double power) {
        wristIntakeMotor.set(ControlMode.PercentOutput, (invertForCone ? -1 : 1) * power);
    }

    /**
     * Releases GPs in intake (if any). Primarily used to prevent the accidental holding of multiple
     * GPs penalty.
     */
    public void panic() {
        setMotor(Constants.Wrist.INTAKE_PANIC_SPEED);
    }

    /*
     * Get if motor is stalling by checking if the stator current exceeds a constant
     */
    public boolean getVoltageSpike(boolean passedTime) {
        return wristIntakeMotor.getStatorCurrent() > Constants.Wrist.STALL_CURRENT;
    }

    public void setInvert(boolean invert) {
        invertForCone = invert;
    }
}
