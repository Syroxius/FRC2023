package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

/**
 * This command will move the arm to a requested angle.
 */
public class MoveElevator extends CommandBase {
    private Arm arm;
    private double elevatorPosition;

    /**
     * Requirements for the command.
     *
     * @param arm Arm subsystem.
     * @param elevatorPosition Desired elevator position
     */
    public MoveElevator(Arm arm, double elevatorPosition) {
        this.arm = arm;
        this.elevatorPosition = MathUtil.clamp(elevatorPosition, 0, Constants.Elevator.MAX_ENCODER);
        addRequirements(arm);
    }

    /**
     * Requirements for the command.
     *
     * @param arm Arm subsystem.
     */
    public MoveElevator(Arm arm) {
        this(arm, 0);
    }

    @Override
    public void initialize() {
        arm.enablePID();
        // arm.setElevatorGoal(elevatorPosition);
    }

    @Override
    public boolean isFinished() {
        return arm.checkElevatorAligned();
    }
}
