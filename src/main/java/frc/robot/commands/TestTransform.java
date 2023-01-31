package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Test April tag transform
 */
public class TestTransform extends CommandBase {

    private Swerve swerve;
    private Transform2d transform2d;

    HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(Constants.SwerveTransformPID.PID_XKP,
            Constants.SwerveTransformPID.PID_XKI, Constants.SwerveTransformPID.PID_XKD),
        new PIDController(Constants.SwerveTransformPID.PID_YKP,
            Constants.SwerveTransformPID.PID_YKI, Constants.SwerveTransformPID.PID_YKD),
        new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP,
            Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
            new TrapezoidProfile.Constraints(Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));
    Pose2d pose2d = new Pose2d();
    private int aprilTagId;

    /**
     * Test April tag transform
     */
    public TestTransform(Swerve swerve, Transform2d transform2d, int aprilTagId) {

        super();
        this.swerve = swerve;
        this.transform2d = transform2d;
        this.addRequirements(swerve);
        this.aprilTagId = aprilTagId;
        holonomicDriveController.setTolerance(new Pose2d(.01, .01, Rotation2d.fromDegrees(1)));
    }

    @Override
    public void initialize() {
        pose2d = FieldConstants.aprilTags.get(aprilTagId).toPose2d().plus(transform2d);
    }

    @Override
    public void execute() {
        ChassisSpeeds ctrlEffort =
            holonomicDriveController.calculate(swerve.getPose(), pose2d, 0, pose2d.getRotation());
        swerve.setModuleStates(ctrlEffort);
    }

    @Override
    public boolean isFinished() {
        return holonomicDriveController.atReference();
    }
}
