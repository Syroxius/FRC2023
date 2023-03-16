package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/**
 * This command will move the arm to a requested angle.
 */
public class ArmMoving extends CommandBase {
    private Arm arm;
    private double goal;

    /**
     * Requirements for the command.
     *
     * @param arm Arm subsystem.
     * @param goal Goal at which the arm should move to.
     */
    public ArmMoving(Arm arm, double goal) {
        this.arm = arm;
        this.goal = goal;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // arm.enablePID();
        arm.setArmGoal(goal);
    }

    @Override
    public boolean isFinished() {
        return false; // arm.checkArmInPosition();
    }
}
