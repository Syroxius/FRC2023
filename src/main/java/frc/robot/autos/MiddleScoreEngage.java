package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.DockArm;
import frc.robot.commands.drive.MoveToPos;
import frc.robot.commands.drive.MoveToScore;
import frc.robot.commands.wrist.WristIntakeRelease;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;

/**
 * Score one game piece then engage from community side.
 */
public class MiddleScoreEngage extends SequentialCommandGroup {

    Swerve swerve;
    Pose2d aprilTag7 = FieldConstants.aprilTags.get(7).toPose2d();

    /**
     * Auto constructor
     *
     * @param swerve Swerve
     * @param arm Arm
     * @param wristIntake Wrist Intake
     */
    public MiddleScoreEngage(Swerve swerve, Arm arm, WristIntake wristIntake) {
        this.swerve = swerve;
        MoveToScore moveToScore = new MoveToScore(swerve, arm, wristIntake);
        ParallelRaceGroup wristIntakeRelease = new WristIntakeRelease(wristIntake).withTimeout(.5);
        MoveToPos move7 = new MoveToPos(swerve, () -> get7Position(), true);
        ParallelRaceGroup dockArm = new DockArm(arm, wristIntake).withTimeout(1);
        ClimbPlatformAuto climbPlatform = new ClimbPlatformAuto(swerve);
        ConditionalCommand toDockOrNotToDock = new ConditionalCommand(climbPlatform,
            new InstantCommand(), () -> RobotContainer.enableDockWidget.getBoolean(true));

        addCommands(moveToScore, wristIntakeRelease,
            move7.alongWith(new WaitCommand(.5).andThen(dockArm)), toDockOrNotToDock);
    }

    private Pose2d get7Position() {
        double x = aprilTag7.getX() + Units.inchesToMeters(50);
        double y = aprilTag7.getY();
        return new Pose2d(x, y, Rotation2d.fromDegrees(180));
    }
}
