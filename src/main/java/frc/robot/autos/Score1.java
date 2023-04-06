package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.Scoring;
import frc.robot.commands.arm.DockArm;
import frc.robot.commands.drive.MoveToPos;
import frc.robot.commands.drive.MoveToScore;
import frc.robot.commands.wrist.WristIntakeRelease;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;

/**
 * Score1Dock Auto.
 */
public class Score1 extends SequentialCommandGroup {

    Swerve swerve;

    /**
     * Auto constructor
     *
     * @param swerve Swerve
     * @param arm Arm
     * @param wristIntake Wrist Intake
     */
    public Score1(Swerve swerve, Arm arm, WristIntake wristIntake) {
        this.swerve = swerve;
        MoveToScore moveToScore = new MoveToScore(swerve, arm, wristIntake);
        ParallelRaceGroup wristIntakeRelease = new WristIntakeRelease(wristIntake).withTimeout(.5);
        MoveToPos moveToPos = new MoveToPos(swerve, Scoring::getPreScorePosition, false);
        ParallelRaceGroup dockArm = new DockArm(arm, wristIntake).withTimeout(1);

        addCommands(moveToScore, wristIntakeRelease,
            moveToPos.alongWith(new WaitCommand(.5).andThen(dockArm)));
    }



}
