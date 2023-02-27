package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.WristIntake;

/**
 * Command to dock the arm in the robot
 */
public class DockArm extends SequentialCommandGroup {

    /**
     * Command to dock the arm in the robot
     *
     * @param arm Arm subsystem.
     * @param wristIntake Wrist Intake subsystem
     */
    public DockArm(Arm arm, WristIntake wristIntake) {
        addRequirements(arm);

        // MoveDDIntake moveDDIntake = new MoveDDIntake(dIntake, dIntake.position1);
        InstantCommand closeGrabber = new InstantCommand(() -> wristIntake.closeGrabber());
        WaitCommand waitCommand = new WaitCommand(.25);
        WaitCommand waitCommand2 = new WaitCommand(.25);
        // MoveElevator moveElevator = new MoveElevator(arm, 0);
        MoveArm moveArm1 = new MoveArm(arm, () -> new ArmPosition(20, 0, -10));
        MoveArm moveArm2 = new MoveArm(arm, () -> new ArmPosition(-5, 0, -10));
        // MoveDDIntake moveDDIntake2 = new MoveDDIntake(dIntake, dIntake.position3);

        addCommands(closeGrabber, waitCommand, moveArm1, waitCommand2, moveArm2);
    }

}
