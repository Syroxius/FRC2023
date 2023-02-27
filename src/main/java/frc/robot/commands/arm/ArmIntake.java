package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/**
 * This command will move the arm to a requested angle.
 */
public class ArmIntake extends CommandBase {
    private Arm arm;
    private double armAngle = 20;
    private double wristAngle = 20;

    /**
     * Requirements for the command.
     *
     * @param arm Arm subsystem.
     */
    public ArmIntake(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.enablePID();
        arm.setArmGoal(armAngle);
        arm.setWristOffset(wristAngle);
        // arm.setElevatorGoal(elevatorPosition);
    }

    @Override
    public boolean isFinished() {
        return arm.checkArmInPosition();
    }
}
