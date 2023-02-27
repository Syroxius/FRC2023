package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.ClimbPlatform;
import frc.robot.subsystems.Swerve;

/**
 * Auto to climb the platform
 */
public class ClimbPlatformAuto extends SequentialCommandGroup {

    /**
     * Auto to climb the platform
     *
     * @param swerve Swerve Drive subsystem
     */
    public ClimbPlatformAuto(Swerve swerve) {
        addRequirements(swerve);


        // MoveToPos rotate = new MoveToPos(swerve)
        ClimbPlatform climb = new ClimbPlatform(swerve);

        addCommands(climb);
    }

}
