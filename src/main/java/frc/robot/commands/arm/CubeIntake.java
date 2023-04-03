package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.ArmPosition;
import frc.robot.subsystems.Arm;

/**
 * This command will move the arm to a requested angle.
 */
public class CubeIntake extends SequentialCommandGroup {
    public static final double armAngle = -48.0;
    public static final double wristAngle = -69.0;

    /**
     * Requirements for the command.
     *
     * @param arm Arm subsystem.
     */
    public CubeIntake(Arm arm) {
        addRequirements(arm);
        MoveArm moveArm =
            new MoveArm(arm, () -> new ArmPosition(armAngle, false, DockArm.wristAngle));
        MoveArm moveArm2 = new MoveArm(arm, () -> new ArmPosition(armAngle, false, wristAngle));
        addCommands(new ConditionalCommand(moveArm.withTimeout(1.0), new InstantCommand(),
            () -> arm.getArmAngle() < CubeIntake.armAngle - 5), moveArm2);
    }
}
