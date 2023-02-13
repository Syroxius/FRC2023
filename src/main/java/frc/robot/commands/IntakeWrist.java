package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristIntake;

/**
 * This command will enable the intake on the wrist and then stop the intake once a set of touch
 * sensors are being touched.
 */
public class IntakeWrist extends CommandBase {
    private WristIntake wIntake;

    public IntakeWrist(WristIntake wristIntake) {
        this.wIntake = wristIntake;
        addRequirements(wIntake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        wIntake.intake();
    }

    @Override
    public void end(boolean interrupt) {
        wIntake.stop();
    }

    @Override
    public boolean isFinished() {
        return wIntake.getConeSensor() || wIntake.getCubeSensor();
    }
}
