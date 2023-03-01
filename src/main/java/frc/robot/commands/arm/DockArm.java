package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.WristIntake;

/**
 * Command to dock the arm in the robot
 */
public class DockArm extends SequentialCommandGroup {

    Arm arm;

    /**
     * Command to dock the arm in the robot
     *
     * @param arm Arm subsystem.
     * @param wristIntake Wrist Intake subsystem
     */
    public DockArm(Arm arm, WristIntake wristIntake) {
        this.arm = arm;
        addRequirements(arm);

        InstantCommand closeGrabber = new InstantCommand(() -> wristIntake.closeGrabber());
        MoveArm moveArm1 = new MoveArm(arm, () -> new ArmPosition(20, 0, -10));
        ConditionalCommand cond =
            new ConditionalCommand(moveArm1, new InstantCommand(), () -> armInside());
        MoveArm moveArm2 = new MoveArm(arm, () -> new ArmPosition(-5, 0, -10));

        addCommands(closeGrabber, cond, moveArm2);
    }

    private boolean armInside() {
        return (arm.getAverageArmAngle() > 180 ? arm.getAverageArmAngle() - 360
            : arm.getAverageArmAngle()) > 15;
    }

}
