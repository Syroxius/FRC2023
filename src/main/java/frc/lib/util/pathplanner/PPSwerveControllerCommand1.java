package frc.lib.util.pathplanner;

import java.util.function.Consumer;
import java.util.function.Supplier;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Command which properly transforms our path in our convention.
 */
public class PPSwerveControllerCommand1 extends PPSwerveControllerCommand {
    private PathPlannerTrajectory trajectory;

    /**
     * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
     * trajectory. This command will not return output voltages but ChassisSpeeds from the position
     * controllers which need to be converted to module states and put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the output to zero upon completion of the path this is
     * left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory The trajectory to follow.
     * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
     *        to provide this.
     * @param xController The Trajectory Tracker PID controller for the robot's x position.
     * @param yController The Trajectory Tracker PID controller for the robot's y position.
     * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
     * @param outputChassisSpeeds The field relative chassis speeds output consumer.
     * @param requirements The subsystems to require.
     */
    public PPSwerveControllerCommand1(PathPlannerTrajectory trajectory,
        Supplier<Pose2d> poseSupplier, PIDController xController, PIDController yController,
        PIDController rotationController, Consumer<ChassisSpeeds> outputChassisSpeeds,
        Subsystem... requirements) {
        super(trajectory, poseSupplier, xController, yController, rotationController,
            outputChassisSpeeds, requirements);
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        this.trajectory = PathPlannerTrajectory1.transformTrajectoryForAlliance(trajectory,
            DriverStation.getAlliance());
        super.initialize();
    }
}
