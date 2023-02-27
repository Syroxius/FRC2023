package frc.robot.commands.arm;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.ArmPosition;
import frc.lib.util.Scoring;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.WristIntake;

/**
 * Command to move arm into scoring position
 */
public class ScoreArm extends SequentialCommandGroup {

    Supplier<ArmPosition> getScoreParameters = () -> Scoring.getScoreParameters();

    // /**
    // * Requirements for the command.
    // *
    // * @param arm Arm subsystem.
    // * @param dIntake Drop Down intake subsystem
    // * @param wristIntake Wrist Intake subsystem
    // */
    // public ScoreArm(Arm arm, DropIntake dIntake, WristIntake wristIntake) {
    // addRequirements(arm, dIntake, wristIntake);

    // InstantCommand closeGrabber = new InstantCommand(() -> wristIntake.closeGrabber());
    // WaitCommand waitForGrabber = new WaitCommand(.5);
    // MoveElevator moveElevator = new MoveElevator(arm, 0);
    // SequentialCommandGroup part1 = closeGrabber.andThen(waitForGrabber).andThen(moveElevator);
    // MoveArm moveArmFinal = new MoveArm(arm, getScoreParameters);
    // ConditionalCommand condition = new ConditionalCommand(part1.andThen(moveArmFinal),
    // moveArmFinal, () -> arm.getAverageArmAngle() < 20);

    // addCommands(condition);
    // }

    /**
     * Requirements for the command.
     *
     * @param arm Arm subsystem.
     * @param wristIntake Wrist Intake subsystem
     */
    public ScoreArm(Arm arm, WristIntake wristIntake) {
        addRequirements(arm, wristIntake);

        InstantCommand closeGrabber = new InstantCommand(() -> wristIntake.closeGrabber());
        WaitCommand waitForGrabber = new WaitCommand(.5);
        // MoveElevator moveElevator = new MoveElevator(arm, 0);
        SequentialCommandGroup part1 = new SequentialCommandGroup(closeGrabber, waitForGrabber);
        MoveArm moveArmFinal = new MoveArm(arm, getScoreParameters);
        // SequentialCommandGroup test = new SequentialCommandGroup(part1, moveArmFinal);
        // ConditionalCommand condition =
        // new ConditionalCommand(test, moveArmFinal, () -> arm.getAverageArmAngle() < 20);

        addCommands(part1, moveArmFinal);
    }
}
