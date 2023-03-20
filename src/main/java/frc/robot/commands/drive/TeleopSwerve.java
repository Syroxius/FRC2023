package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

/**
 * Creates an command for driving the swerve drive during tele-op
 */
public class TeleopSwerve extends CommandBase {

    private boolean fieldRelative;
    private boolean openLoop;
    private Swerve swerveDrive;
    private CommandXboxController controller;
    private Arm arm;

    /**
     * Creates an command for driving the swerve drive during tele-op
     *
     * @param swerveDrive The instance of the swerve drive subsystem
     * @param fieldRelative Whether the movement is relative to the field or absolute
     * @param openLoop Open or closed loop system
     */
    public TeleopSwerve(Swerve swerveDrive, CommandXboxController controller, boolean fieldRelative,
        boolean openLoop, Arm arm) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.controller = controller;
        this.arm = arm;
    }

    @Override
    public void execute() {
        double yaxis = -controller.getLeftY();
        double xaxis = -controller.getLeftX();
        double raxis = -controller.getRightX();

        /* Deadbands */
        // yaxis = MathUtil.applyDeadband(yaxis, Constants.STICK_DEADBAND);
        yaxis = Conversions.applySwerveCurve(yaxis, Constants.STICK_DEADBAND);
        // xaxis = MathUtil.applyDeadband(xaxis, Constants.STICK_DEADBAND);
        xaxis = Conversions.applySwerveCurve(xaxis, Constants.STICK_DEADBAND);
        raxis = MathUtil.applyDeadband(raxis, Constants.STICK_DEADBAND);

        double angle_speed = Constants.Swerve.MAX_ANGULAR_VELOCITY;
        double speed = Constants.Swerve.MAX_SPEED;
        if (arm.getArmAngle() > -20) {
            angle_speed /= 2;
            speed *= 0.60;
        }

        Translation2d translation = new Translation2d(yaxis, xaxis).times(speed);
        double rotation = raxis * angle_speed;
        swerveDrive.driveWithTwist(translation, rotation, fieldRelative, openLoop);

    }
}
