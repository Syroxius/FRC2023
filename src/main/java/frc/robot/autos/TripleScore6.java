package frc.robot.autos;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.CubeIntake;
import frc.robot.commands.arm.DockArm;
import frc.robot.commands.wrist.AutoWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;

/**
 * Score three game pieces
 */
public class TripleScore6 extends TrajectoryBase {
    private double maxVel = 4;
    private double maxAccel = 4;
    // private double armAngle = -60.0;
    // private double wristAngle = 3.0;

    /**
     * Score three game pieces
     */
    public TripleScore6(Swerve swerve, Arm arm, WristIntake intake) {
        super(swerve);

        SecondGamePiece6 doubleScore = new SecondGamePiece6(swerve, arm, intake);
        List<PathPlannerTrajectory> trajectory6Group =
            PathPlanner.loadPathGroup("TripleScore6", maxVel, maxAccel);
        PathPlannerTrajectory trajectory6 = trajectory6Group.get(1);
        PPSwerveControllerCommand tripleScore6 = baseSwerveCommand(trajectory6);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Intake In2",
            new CubeIntake(arm).alongWith(AutoWrist.cubeIntake(intake, 1.0).withTimeout(3.0)));
        eventMap.put("Go Home2", new DockArm(arm, intake).withTimeout(0.6));
        eventMap.put("Release Intake2", AutoWrist.cubeOuttake(intake, 0.5).withTimeout(0.6));
        eventMap.put("Home2", new DockArm(arm, intake).withTimeout(0.6));
        FollowPathWithEvents tripleScore8Events =
            new FollowPathWithEvents(tripleScore6, trajectory6.getMarkers(), eventMap);
        // AutoWrist whipIt = AutoWrist.cubeOuttake(intake, 1);

        addCommands(AutoWrist.cubeOuttake(intake).withTimeout(0.3), doubleScore,
            tripleScore8Events);
    }

}
