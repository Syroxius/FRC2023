package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;


/**
 * Move into scoring position
 */
public class MoveToEngage extends SequentialCommandGroup {
    Swerve swerve;

    /**
     * Move into scoring position
     */
    public MoveToEngage(Swerve swerve, Arm arm, WristIntake wristIntake) {
        this.swerve = swerve;
        // ConditionalCommand cond = new ConditionalCommand(new TurnToAngle(swerve, 0, false),
        // new TurnToAngle(swerve, 180, false), () -> centerField(swerve));

        ClimbPlatform climbPlatform = new ClimbPlatform(swerve);
        addCommands(climbPlatform);
    }

    /**
     * Check if the robot's position is inside the community or center of the field
     *
     * @return True if the robot is in the middle of the field. False if in the community
     */
    private boolean centerField() {
        Pose2d pose = FieldConstants.allianceFlip(swerve.getPose());
        return pose.getX() > FieldConstants.Community.cableBumpInnerX;
    }
}
