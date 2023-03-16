package frc.robot.commands.arm;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.ArmPosition;
import frc.robot.subsystems.Arm;

/**
 * This command will move the arm to a requested angle.
 */
public class MoveArm extends CommandBase {
    private Arm arm;
    private Supplier<ArmPosition> armPositionSupplier;
    private double armAngle;
    private boolean armExtended;
    private double wristAngle;

    /**
     * Requirements for the command.
     *
     * @param arm Arm subsystem.
     * @param armAngle Angle at which the arm should move to.
     */
    public MoveArm(Arm arm, double armAngle, boolean armExtended) {
        this.arm = arm;
        this.armAngle = armAngle;
        this.armExtended = armExtended;
        addRequirements(arm);
    }

    /**
     * Requirements for the command.
     *
     * @param arm Arm subsystem.
     * @param positionSupplier Supplier of ArmPosition
     */
    public MoveArm(Arm arm, Supplier<ArmPosition> positionSupplier) {
        this.arm = arm;
        this.armPositionSupplier = positionSupplier;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        ArmPosition position = armPositionSupplier.get();
        this.wristAngle = position.getWristAngle();
        this.armAngle = position.getArmAngle();
        arm.setArmGoal(armAngle);
        arm.setWristGoal(wristAngle);
        // arm.setWristGoal(-25);
        this.armExtended = position.getArmExtended();

        if (!armExtended) {
            arm.retractArm();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (armExtended) {
            arm.extendArm();
        }
    }

    @Override
    public boolean isFinished() {
        return arm.armInPosition() && arm.wristInPosition();
    }
}
