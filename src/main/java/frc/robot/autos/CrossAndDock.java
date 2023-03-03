package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.DockArm;
import frc.robot.commands.drive.MoveToEngage;
import frc.robot.commands.drive.MoveToPos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;

/**
 * Leaves community, and docks.
 */
public class CrossAndDock extends SequentialCommandGroup {


    /**
     * Sequential command group to leave community area, and dock.
     *
     * @param swerve Swerve
     * @param arm Arm
     * @param wrist Wrist
     */
    public CrossAndDock(Swerve swerve, Arm arm, WristIntake wrist) {
        LeaveCommunity leaveCommunity = new LeaveCommunity(swerve);
        ParallelRaceGroup dockArm = new DockArm(arm, wrist).withTimeout(1);
        MoveToPos centerWithDock = new MoveToPos(swerve,
            () -> new Pose2d(
                new Translation2d(Units.inchesToMeters(LeaveCommunity.distanceFromTape),
                    FieldConstants.Grids.lowTranslations[4].getY()),
                new Rotation2d(0)),
            true);
        MoveToEngage engage = new MoveToEngage(swerve, arm, wrist);
        ConditionalCommand toDockOrNotToDock =
            new ConditionalCommand(centerWithDock.andThen(engage), new InstantCommand(),
                () -> RobotContainer.enableDockWidget.getBoolean(true));


        addCommands(leaveCommunity.alongWith(dockArm), toDockOrNotToDock);
    }
}
