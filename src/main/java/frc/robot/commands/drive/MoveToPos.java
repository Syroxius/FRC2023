package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Test April tag transform
 */
public class MoveToPos extends CommandBase {

    public Swerve swerve;
    public Pose2d pose2d = new Pose2d();
    public Pose2d finalPose2d = new Pose2d();

    HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(Constants.SwerveTransformPID.PID_XKP,
            Constants.SwerveTransformPID.PID_XKI, Constants.SwerveTransformPID.PID_XKD),
        new PIDController(Constants.SwerveTransformPID.PID_YKP,
            Constants.SwerveTransformPID.PID_YKI, Constants.SwerveTransformPID.PID_YKD),
        new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP,
            Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
            new TrapezoidProfile.Constraints(Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));

    /**
     * Test April tag transform
     */
    public MoveToPos(Swerve swerve) {
        super();
        this.swerve = swerve;
        this.addRequirements(swerve);
        holonomicDriveController.setTolerance(new Pose2d(.01, .01, Rotation2d.fromDegrees(1)));
    }

    /**
     * Test April tag transform
     */
    public MoveToPos(Swerve swerve, Pose2d position) {
        this(swerve);
        this.pose2d = position;
    }

    @Override
    public void initialize() {
        finalPose2d = null;
        finalPose2d = pose2d;
        if (DriverStation.getAlliance() == Alliance.Red) {
            var translation =
                new Translation2d(FieldConstants.fieldLength - pose2d.getX(), pose2d.getY());
            var rotation = pose2d.getRotation().plus(Rotation2d.fromDegrees(180));
            finalPose2d = new Pose2d(translation, rotation);
        }
    }

    @Override
    public void execute() {
        ChassisSpeeds ctrlEffort = holonomicDriveController.calculate(swerve.getPose(), finalPose2d,
            0, finalPose2d.getRotation());
        swerve.setModuleStates(ctrlEffort);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setMotorsZero();
    }

    @Override
    public boolean isFinished() {
        return holonomicDriveController.atReference();
    }
}
