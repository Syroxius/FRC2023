package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.pathplanner.PPSwerveControllerCommand1;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Parent class for all autonomous commands
 */
public class TrajectoryBase extends SequentialCommandGroup {
    public Swerve swerve;
    private PIDController pidX = new PIDController(Constants.SwerveTransformPID.PID_XKP,
        Constants.SwerveTransformPID.PID_XKI, Constants.SwerveTransformPID.PID_XKD);
    private PIDController pidY = new PIDController(Constants.SwerveTransformPID.PID_YKP,
        Constants.SwerveTransformPID.PID_YKI, Constants.SwerveTransformPID.PID_YKD);
    private PIDController pidTheta = new PIDController(Constants.SwerveTransformPID.PID_TKP,
        Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD);

    /**
     * Autonomous that aligns limelight then executes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public TrajectoryBase(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
        pidTheta.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Creates a SwerveController Command using a Path Planner Trajectory
     *
     * @param trajectory a Path Planner Trajectory
     * @return A SwerveControllerCommand for the robot to move
     */
    public PPSwerveControllerCommand1 baseSwerveCommand(PathPlannerTrajectory trajectory) {
        PPSwerveControllerCommand1 command = new PPSwerveControllerCommand1(trajectory,
            swerve::getPose, pidX, pidY, pidTheta, swerve::setModuleStates, swerve);
        return command;
    }
}
