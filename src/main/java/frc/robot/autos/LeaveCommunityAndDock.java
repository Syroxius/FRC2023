package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.MoveToEngage;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;

/**
 * Leaves community area.
 */
public class LeaveCommunityAndDock extends TrajectoryBase {

    private double maxVel = 6.0;
    private double maxAccel = 3.0;

    /**
     * Leaves community
     *
     * @param swerve swerve
     */
    public LeaveCommunityAndDock(Swerve swerve, Arm arm, WristIntake wrist) {
        super(swerve);
        PathPlannerTrajectory trajectory8 =
            PathPlanner.loadPath("Leave Community - 8", maxVel, maxAccel);
        PathPlannerTrajectory trajectory6 =
            PathPlanner.loadPath("Leave Community - 6", maxVel, maxAccel);
        PPSwerveControllerCommand leaveCommunityAndCross8 = baseSwerveCommand(trajectory8);
        PPSwerveControllerCommand leaveCommunityAndCross6 = baseSwerveCommand(trajectory6);

        MoveToEngage engage = new MoveToEngage(swerve, arm, wrist);
        ConditionalCommand toDockOrNotToDock8 = new ConditionalCommand(leaveCommunityAndCross8,
            new LeaveCommunity(swerve), () -> RobotContainer.enableDockWidget.getBoolean(true));
        ConditionalCommand toDockOrNotToDock6 = new ConditionalCommand(leaveCommunityAndCross6,
            new LeaveCommunity(swerve), () -> RobotContainer.enableDockWidget.getBoolean(true));
        ConditionalCommand cond =
            new ConditionalCommand(toDockOrNotToDock6, toDockOrNotToDock8, () -> chooseSide());
        ConditionalCommand cond1 = new ConditionalCommand(engage, new InstantCommand(),
            () -> RobotContainer.enableDockWidget.getBoolean(true));

        addCommands(cond, cond1);
    }

    /**
     * Boolean to choose which side to cross community zone
     *
     * @return True if closer to feeder station, False if closer to Scoring Table
     */
    private boolean chooseSide() {
        Pose2d pose2d = swerve.getPose();
        double midPlatform =
            Units.inchesToMeters(59.39) + FieldConstants.Community.chargingStationWidth / 2;
        return pose2d.getY() > midPlatform;
    }
}
