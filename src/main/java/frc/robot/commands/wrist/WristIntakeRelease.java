package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.WristIntake;

/**
 * Command to raise the Drop Down Intake to the top position
 */
public class WristIntakeRelease extends ParallelCommandGroup {

    /**
     * Command to lower the Drop Down Intake to the bottom position
     *
     * @param intake Drop Down Intake
     */
    public WristIntakeRelease(WristIntake intake) {
        addRequirements(intake);

        StartEndCommand startMotors =
            new StartEndCommand(() -> intake.setMotors(Constants.Wrist.INTAKE_RELEASE_SPEED),
                () -> intake.setMotors(Constants.Wrist.INTAKE_STOP_SPEED));
        InstantCommand openGrabber = new InstantCommand(() -> intake.openGrabber());

        addCommands(startMotors, new WaitCommand(.5).andThen(openGrabber));

    }
}
