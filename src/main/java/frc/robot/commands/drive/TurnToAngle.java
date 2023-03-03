package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 * This command will turn the robot to a specified angle.
 */
public class TurnToAngle extends CommandBase {

    private Swerve swerve;
    private boolean isRelative;
    private double goal;
    private HolonomicDriveController holonomicDriveController;
    private Pose2d startPos = new Pose2d();
    private Pose2d targetPose2d = new Pose2d();
    private double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
    private double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

    /**
     * Turns robot to specified angle. Uses absolute rotation on field.
     *
     * @param swerve Swerve subsystem
     * @param angle Requested angle to turn to
     * @param isRelative Whether the angle is relative to the current angle: true = relative, false
     *        = absolute
     */

    public TurnToAngle(Swerve swerve, double angle, boolean isRelative) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.goal = angle;
        this.isRelative = isRelative;

        PIDController xcontroller = new PIDController(0, 0, 0);
        PIDController ycontroller = new PIDController(0, 0, 0);
        ProfiledPIDController thetacontroller =
            new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared));
        holonomicDriveController =
            new HolonomicDriveController(xcontroller, ycontroller, thetacontroller);
        holonomicDriveController.setTolerance(new Pose2d(1, 1, Rotation2d.fromDegrees(0.5)));

    }

    @Override
    public void initialize() {
        startPos = swerve.getPose();
        if (isRelative) {
            targetPose2d = new Pose2d(startPos.getTranslation(),
                startPos.getRotation().rotateBy(Rotation2d.fromDegrees(goal)));
        } else {
            targetPose2d = new Pose2d(startPos.getTranslation(), Rotation2d.fromDegrees(goal));
        }
        if (DriverStation.getAlliance() == Alliance.Red) {
            targetPose2d = new Pose2d(targetPose2d.getTranslation(),
                targetPose2d.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        }
    }

    @Override
    public void execute() {
        Pose2d currPose2d = swerve.getPose();
        ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d,
            targetPose2d, 0, targetPose2d.getRotation());
        swerve.setModuleStates(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupt) {
        swerve.setMotorsZero();
    }

    @Override
    public boolean isFinished() {
        return holonomicDriveController.atReference();

    }
}
