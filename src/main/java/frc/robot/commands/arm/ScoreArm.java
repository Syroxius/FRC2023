package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.Scoring;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.WristIntake;

/**
 * Command to move arm into scoring position
 */
public class ScoreArm extends SequentialCommandGroup {

    /**
     * Requirements for the command.
     *
     * @param arm Arm subsystem.
     * @param wristIntake Wrist Intake subsystem
     */
    public ScoreArm(Arm arm, WristIntake wristIntake) {
        addRequirements(arm, wristIntake);

        MoveArm moveArmFinal = new MoveArm(arm, Scoring::getScoreParameters);

        addCommands(moveArmFinal);
    }
}
