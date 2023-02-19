package frc.robot.commands.dropintake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DropIntake;

/**
 * Command to raise the Drop Down Intake to the top position
 */
public class MoveDDIntake extends CommandBase {
    private DropIntake intake;
    private double angle;

    /**
     * Command to lower the Drop Down Intake to the bottom position
     *
     * @param intake Drop Down Intake
     */
    public MoveDDIntake(DropIntake intake, double angle) {
        this.intake = intake;
        this.angle = angle;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.enablePID();
        intake.setGoal(angle);
    }

    @Override
    public boolean isFinished() {
        // return intake.checkIfAligned(216.0);
        return intake.checkIfAligned(angle);
    }
}
