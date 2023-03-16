package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WristIntake;

/**
 * Command to raise the Drop Down Intake to the top position
 */
public class WristIntakeRelease extends CommandBase {

    WristIntake mIntake;

    /**
     * Command to lower the Drop Down Intake to the bottom position
     *
     * @param intake Drop Down Intake
     */
    public WristIntakeRelease(WristIntake intake) {
        addRequirements(intake);
        mIntake = intake;
    }

    @Override
    public void initialize() {
        mIntake.stopHoldingPiece();
        mIntake.setMotor(Constants.Wrist.INTAKE_RELEASE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.setMotor(Constants.Wrist.INTAKE_STOP_SPEED);
    }


}
