package frc.robot.autos;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ArmPosition;
import frc.lib.util.FieldConstants;
import frc.robot.commands.arm.CubeIntake;
import frc.robot.commands.arm.DockArm;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.wrist.AutoWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;


/**
 * Acquire a second game piece
 */
public class SecondGamePiece6 extends TrajectoryBase {

    private double maxVel = 4;
    private double maxAccel = 3;
    // private double armAngle = -60.0;
    // private double wristAngle = 3.0;
    Pose2d aprilTag8 = FieldConstants.aprilTags.get(8).toPose2d();
    Pose2d aprilTag6 = FieldConstants.aprilTags.get(6).toPose2d();

    /**
     * Acquire a second game piece
     */
    public SecondGamePiece6(Swerve swerve, Arm arm, WristIntake intake) {
        super(swerve);
        List<PathPlannerTrajectory> trajectory6Group =
            PathPlanner.loadPathGroup("TripleScore6", maxVel, maxAccel);
        PathPlannerTrajectory trajectory6 = trajectory6Group.get(0);
        PPSwerveControllerCommand secondGamePiece6 = baseSwerveCommand(trajectory6);
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("start",
            new MoveArm(arm, () -> new ArmPosition(CubeIntake.armAngle, false, DockArm.wristAngle))
                .withTimeout(2.0));
        eventMap.put("Intake On",
            AutoWrist.cubeIntake(intake, 0.8)
                .alongWith(new MoveArm(arm,
                    () -> new ArmPosition(CubeIntake.armAngle, false, CubeIntake.wristAngle)))
                .withTimeout(4.0));
        eventMap.put("Go Home", new DockArm(arm, intake));
        eventMap.put("Release Intake", AutoWrist.cubeOuttake(intake, 0.3).withTimeout(1.2));
        FollowPathWithEvents secondGamePiece6Events =
            new FollowPathWithEvents(secondGamePiece6, trajectory6.getMarkers(), eventMap);
        addCommands(secondGamePiece6Events);

    }

}

