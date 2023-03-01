package frc.robot.commands.wrist;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.Scoring;
import frc.lib.util.Scoring.GamePiece;
import frc.robot.Constants;
import frc.robot.subsystems.WristIntake;

/**
 * Command to raise the Drop Down Intake to the top position
 */
public class WristIntakeRelease extends ParallelCommandGroup {
    Supplier<GamePiece> getGamePiece = () -> Scoring.getGamePiece();

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
        ConditionalCommand condition =
            new ConditionalCommand(startMotors, new InstantCommand(), Scoring::gamePieceIsCube);
        SequentialCommandGroup openGrabber = new SequentialCommandGroup(new WaitCommand(.25),
            new InstantCommand(() -> intake.openGrabber()));

        addCommands(condition, openGrabber);

    }
}
