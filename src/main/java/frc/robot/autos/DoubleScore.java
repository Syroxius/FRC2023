package frc.robot.autos;

import java.util.HashMap;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.DockArm;
import frc.robot.commands.drive.MoveToPos;
import frc.robot.commands.wrist.AutoWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;

/**
 * Score a second game piece
 */
public class DoubleScore extends TrajectoryBase {

    private double maxVel = 6;
    private double maxAccel = 4;
    // private double armAngle = -60.0;
    // private double wristAngle = 3.0;
    Pose2d aprilTag7 = FieldConstants.aprilTags.get(7).toPose2d();


    /**
     * Score a second game piece
     */
    public DoubleScore(Swerve swerve, Arm arm, WristIntake intake) {
        this(swerve, arm, intake, true);
    }


    /**
     * Score a second game piece
     */
    public DoubleScore(Swerve swerve, Arm arm, WristIntake intake, boolean isMainCommand) {
        super(swerve);

        PathPlannerTrajectory trajectory8 = PathPlanner.loadPath("DoubleScore8", maxVel, maxAccel);
        PPSwerveControllerCommand doubleScore8 = baseSwerveCommand(trajectory8);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Release Intake", AutoWrist.cubeOuttake(intake, 0.4).withTimeout(1.2));
        // eventMap.put("Go Home", new DockArm(arm, intake));
        FollowPathWithEvents doubleScore8Events =
            new FollowPathWithEvents(doubleScore8, trajectory8.getMarkers(), eventMap);

        MoveToPos move7 = new MoveToPos(swerve, () -> get7Position(), true);
        ParallelRaceGroup dockArm = new DockArm(arm, intake).withTimeout(1);
        ClimbPlatformAuto climbPlatform = new ClimbPlatformAuto(swerve);
        ConditionalCommand toDockOrNotToDock = new ConditionalCommand(
            move7.alongWith(new WaitCommand(.5).andThen(dockArm)).andThen(climbPlatform),
            new InstantCommand(),
            () -> isMainCommand && RobotContainer.enableDockWidget.getBoolean(true));

        if (isMainCommand) {
            addCommands(AutoWrist.cubeOuttake(intake).withTimeout(0.3));
        }

        addCommands(new SecondGamePiece(swerve, arm, intake), doubleScore8Events,
            toDockOrNotToDock);

    }

    private Pose2d get7Position() {
        double x = aprilTag7.getX() + Units.inchesToMeters(50);
        double y = aprilTag7.getY();
        return new Pose2d(x, y, Rotation2d.fromDegrees(180));
    }
}
