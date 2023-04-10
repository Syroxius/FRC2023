package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Test April tag transform
 */
public class ClimbPlatform extends CommandBase {

    private Swerve swerve;
    private boolean beenTilted = false;
    private double startTime;
    private int endCount = 0;

    private PIDController pidController = new PIDController(-.006, 0, 0);

    /**
     * Test April tag transform
     */
    public ClimbPlatform(Swerve swerve) {
        this.swerve = swerve;
        this.addRequirements(swerve);
    }

    @Override
    public void initialize() {
        beenTilted = false;
        startTime = 0;
        endCount = 0;
        pidController.reset();
        pidController.setSetpoint(0);
        pidController.setTolerance(2.0);
    }

    @Override
    public void execute() {
        // && (Math.abs(swerve.getRoll()) < 10 || )
        double speed = -1.5;
        if (!beenTilted && Math.abs(swerve.getRoll()) > 10) {
            beenTilted = true;
            startTime = Timer.getFPGATimestamp();
        }
        if (beenTilted && Timer.getFPGATimestamp() > startTime + .8) {
            speed = pidController.calculate(swerve.getRoll());
        }
        SmartDashboard.putBoolean("Been Tilted", beenTilted);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, 0, 0);
        // chassisSpeeds =
        // ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, swerve.getFieldRelativeHeading());
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(swerveModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.wheelsIn();
    }

    @Override
    public boolean isFinished() {
        if (pidController.atSetpoint()) {
            endCount++;
        } else {
            endCount = 0;
        }
        return endCount > 5;
    }
}
