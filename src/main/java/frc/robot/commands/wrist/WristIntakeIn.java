package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristIntake;

/**
 * Command to raise the Drop Down Intake to the top position
 */
public class WristIntakeIn extends CommandBase {
    private WristIntake intake;

    /**
     * Command to lower the Drop Down Intake to the bottom position
     *
     * @param intake Drop Down Intake
     */
    public WristIntakeIn(WristIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intake();
    }

    @Override
    public void end(boolean interrupt) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        // return intake.getConeSensor() || intake.getCubeSensor();
        return false;
    }
}
