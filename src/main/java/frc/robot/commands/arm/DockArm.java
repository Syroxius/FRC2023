package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.WristIntake;

/**
 * Command to dock the arm in the robot
 */
public class DockArm extends SequentialCommandGroup {

    Arm arm;
    public static final double wristAngle = 67;

    /**
     * Command to dock the arm in the robot
     *
     * @param arm Arm subsystem.
     * @param wristIntake Wrist Intake subsystem
     */
    public DockArm(Arm arm, WristIntake wristIntake) {
        this.arm = arm;
        addRequirements(arm);

        // MoveArm moveArm1 = new MoveArm(arm, () -> new ArmPosition(20, false, -10));
        // ConditionalCommand cond =
        // new ConditionalCommand(moveArm1, new InstantCommand(), () -> armInside());

        addCommands(new ConditionalCommand(
            new MoveArm(arm, () -> new ArmPosition(-82.9, false, wristAngle)),
            new MoveArm(arm, () -> new ArmPosition(CubeIntake.armAngle, false, wristAngle))
                .withTimeout(0.2)
                .andThen(new MoveArm(arm, () -> new ArmPosition(-82.9, false, wristAngle))),
            () -> arm.getWristAngle() > 0));
    }

    // private boolean armInside() {
    // return (arm.getAverageArmAngle() > 180 ? arm.getAverageArmAngle() - 360
    // : arm.getAverageArmAngle()) > 25;
    // }

}
