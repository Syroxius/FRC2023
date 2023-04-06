package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.lib.util.FieldConstants;
import frc.robot.commands.arm.DockArm;
import frc.robot.commands.drive.MoveToPos;
import frc.robot.commands.drive.MoveToScore;
import frc.robot.commands.wrist.WristIntakeRelease;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;

/**
 * Acquire a second game piece after scoring
 */
public class SecondGamePieceScore extends TrajectoryBase {

    Pose2d aprilTag8 = FieldConstants.aprilTags.get(8).toPose2d();

    /**
     * Acquire a second game piece after scoring
     */
    public SecondGamePieceScore(Swerve swerve, Arm arm, WristIntake wristIntake) {
        super(swerve);
        MoveToScore movetoScore = new MoveToScore(swerve, arm, wristIntake);
        ParallelRaceGroup wristIntakeRelease = new WristIntakeRelease(wristIntake).withTimeout(.5);
        SecondGamePiece secondGamePiece = new SecondGamePiece(swerve, arm, wristIntake);
        MoveToPos move8 = new MoveToPos(swerve, () -> get8position(), true, 0.07, 3.0);


        addCommands(movetoScore, wristIntakeRelease,
            move8.alongWith(new DockArm(arm, wristIntake).withTimeout(0.2)), secondGamePiece);

    }

    /**
     * Create the Pose2d closer to scoring table
     *
     * @return Pose2d on scoring table side
     */
    private Pose2d get8position() {
        double x = 2.29743;
        double y = 0.754253;
        return new Pose2d(x, y, Rotation2d.fromDegrees(90));
    }

}
