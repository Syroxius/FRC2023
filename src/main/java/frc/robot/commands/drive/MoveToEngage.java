package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.commands.arm.DockArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;


/**
 * Move into scoring position
 */
public class MoveToEngage extends SequentialCommandGroup {

    /**
     * Move into scoring position
     */
    public MoveToEngage(Swerve swerve, Arm arm, WristIntake wristIntake) {

        ParallelRaceGroup dockArm = new DockArm(arm, wristIntake).withTimeout(2);
        // ConditionalCommand cond = new ConditionalCommand(
        // new MoveToPos(swerve,
        // () -> new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(),
        // new Rotation2d(0)),
        // true),
        // new MoveToPos(swerve, () -> new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(),
        // new Rotation2d(Math.PI)), true),
        // () -> centerField(swerve));
        ClimbPlatform climbPlatform = new ClimbPlatform(swerve);
        addCommands(dockArm, climbPlatform);
    }

    private boolean centerField(Swerve swerve) {
        Pose2d pose = FieldConstants.allianceFlip(swerve.getPose());
        return pose.getX() > Units.inchesToMeters(200);
    }
}
