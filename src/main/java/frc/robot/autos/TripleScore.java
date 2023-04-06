package frc.robot.autos;

import java.util.HashMap;
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
public class TripleScore extends TrajectoryBase {
    private double maxVel = 4;
    private double maxAccel = 4;
    // private double armAngle = -60.0;
    // private double wristAngle = 3.0;

    /**
     * Score three game pieces
     */
    public TripleScore(Swerve swerve, Arm arm, WristIntake intake) {
        super(swerve);

        DoubleScore doubleScore = new DoubleScore(swerve, arm, intake, false);
        PathPlannerTrajectory trajectory8 = PathPlanner.loadPath("TripleScore8", maxVel, maxAccel);
        PPSwerveControllerCommand tripleScore8 = baseSwerveCommand(trajectory8);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Intake In",
            new CubeIntake(arm).alongWith(AutoWrist.cubeIntake(intake, 1.0).withTimeout(3.0)));
        eventMap.put("Go Home", new DockArm(arm, intake).withTimeout(0.6));
        eventMap.put("Release Intake", AutoWrist.cubeOuttake(intake, 0.5).withTimeout(0.6));
        eventMap.put("Home", new DockArm(arm, intake).withTimeout(0.6));
        FollowPathWithEvents tripleScore8Events =
            new FollowPathWithEvents(tripleScore8, trajectory8.getMarkers(), eventMap);
        AutoWrist whipIt = AutoWrist.cubeOuttake(intake, 1);

        addCommands(AutoWrist.cubeOuttake(intake).withTimeout(0.3), doubleScore, tripleScore8Events,
            whipIt.withTimeout(1));

    }

}
