package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.ArmPosition;
import frc.robot.subsystems.Arm;

/**
 * This command will move the arm to a requested angle.
 */
public class ConeUpIntake extends SequentialCommandGroup {
    // private double armAngle = -63.0;
    // private double wristAngle = 18.0;
    public static final double armAngle = -31.0;
    public static final double wristAngle = -47.0;

    /**
     * Requirements for the command.
     *
     * @param arm Arm subsystem.
     */
    public ConeUpIntake(Arm arm) {
        addRequirements(arm);
        MoveArm moveArm2 = new MoveArm(arm, () -> new ArmPosition(armAngle, false, wristAngle));
        MoveArm moveArm =
            new MoveArm(arm, () -> new ArmPosition(CubeIntake.armAngle, false, DockArm.wristAngle));
        addCommands(new ConditionalCommand(moveArm.withTimeout(1.0), new InstantCommand(),
            () -> arm.getArmAngle() < CubeIntake.armAngle - 5), moveArm2);
    }
}
