package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.ArmPosition;
import frc.lib.util.Scoring;
import frc.lib.util.Scoring.GamePiece;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

/**
 * Test April tag transform
 */
public class TestArm extends CommandBase {

    Arm arm;

    public TestArm(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Targeted Level", Robot.level);
        SmartDashboard.putNumber("Targeted Column", Robot.column);
        GamePiece gamePiece = Scoring.getGamePiece();
        SmartDashboard.putString("Targeted Game Piece", gamePiece.toString());
        ArmPosition position = Scoring.getScoreParameters();
        // arm.enablePID();
        // arm.setArmGoal(position.getArmAngle());
        arm.setArmGoal(60);
        // arm.setElevatorGoal(armExtensionValues.get(Robot.level));
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // arm.setGoal(arm.getAngleMeasurement2());
            // Set elevator to stop and hold
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
