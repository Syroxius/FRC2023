package frc.robot.commands.arm;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.ArmPosition;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

/**
 * This command will move the arm to a requested angle.
 */
public class MoveArm extends CommandBase {
    private Arm arm;
    private Supplier<ArmPosition> armPositionSupplier;
    private double armAngle;
    private double elevatorPosition;
    private double wristAngle;

    /**
     * Requirements for the command.
     *
     * @param arm Arm subsystem.
     * @param armAngle Angle at which the arm should move to.
     * @param elevatorPosition Desired Elevator Position
     */
    public MoveArm(Arm arm, double armAngle, double elevatorPosition) {
        this.arm = arm;
        this.armAngle = armAngle;
        this.elevatorPosition = MathUtil.clamp(elevatorPosition, 0, Constants.Elevator.MAX_ENCODER);
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
        this.elevatorPosition =
            MathUtil.clamp(position.getElevatorPosition(), 0, Constants.Elevator.MAX_ENCODER);
        this.wristAngle = position.getWristAngle();
        this.armAngle = position.getArmAngle();
        arm.enablePID();
        arm.setArmGoal(armAngle);
        arm.setWristOffset(wristAngle);
        // arm.setElevatorGoal(elevatorPosition);
    }

    @Override
    public boolean isFinished() {
        return arm.checkArmInPosition();
        // && arm.getWristAligned();
        // arm.checkElevatorAligned();
    }
}
