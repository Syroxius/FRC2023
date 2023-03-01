package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.Scoring;
import frc.robot.commands.arm.ScoreArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;


/**
 * Move into scoring position
 */
public class MoveToScore extends SequentialCommandGroup {

    /**
     * Move into scoring position
     */
    public MoveToScore(Swerve swerve, Arm arm, WristIntake wristIntake) {

        MoveToPos moveToPos = new MoveToPos(swerve, Scoring::getPreScorePosition, false);
        ScoreArm moveArm = new ScoreArm(arm, wristIntake);
        MoveToPos moveToScoreFinal = new MoveToPos(swerve, Scoring::getScorPose2d, false);
        addCommands(moveToPos, moveArm, moveToScoreFinal);
    }
}
